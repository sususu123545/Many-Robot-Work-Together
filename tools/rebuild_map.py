#!/usr/bin/env python3
"""Rebuild the occupancy map in one command: survey, validate, install, reboot.

Why one command
---------------
The 2026-11-09/10 sessions showed the same six manual steps every time, and each
of them has a way to go quietly wrong:

1. stop the localisation stack first, or `map_server` and `slam_toolbox` fight
   over `/map` and the Pi 4 runs out of CPU;
2. upload the four helper files (the container's /tmp is wiped on reboot);
3. start slam_toolbox, wait for `active` (it comes up `unconfigured`);
4. rotate, and **check that the map actually grew** -- the mecanum wheels slip on
   this floor and have reported +720 deg of rotation while the robot never moved;
5. validate the result against a live scan with an independent brute-force fit,
   because "the mapper ran" is not "the map is right";
6. install it with a dated backup, reboot, and confirm the boot path localised.

Anything less and the failure surfaces hours later as "the robot is confidently
in the wrong place".

Usage
-----
    python tools/rebuild_map.py --yes             # full run
    python tools/rebuild_map.py --yes --no-install  # survey + validate only
    python tools/rebuild_map.py --yes --turns 2     # denser, twice the time

Refuses to install a map that does not pass validation.  Refuses to rotate if a
single scan says the sweep is blocked.
"""
import argparse
import base64
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

CORE = 'armpi_ros2_native'
MAPS_DIR = '/opt/armpi-maps'
DEFAULT_HOST = '100.102.57.19'
CREDS = ROOT / 'tools/runtime/arm_creds.json'
TUNNEL_SCRIPT = HERE / 'ssh_tunnel.py'

UPLOADS = [
    ('car_setup/ros2/slam_toolbox_mapping.yaml', 'slam_toolbox_mapping.yaml'),
    ('tools/watch_map_growth.py', 'watch_map_growth.py'),
    ('tools/save_map_in_container.py', 'save_map_in_container.py'),
    ('tools/slam_session.sh', 'slam_session.sh'),
]

# acceptance gates for a freshly built map
MIN_OCCUPIED = 250
MAX_SPECKLE = 0.06          # share of occupied cells that have no occupied neighbour
MAX_FIT_M = 0.12            # mean beam-to-obstacle distance, metres

# What the tool has already done to the car, so an abort can undo the damage:
# stopping localisation and walking away would leave the robot without a
# working pose until somebody reboots it by hand.
STATE = {'client': None, 'localisation_stopped': False}


def log(message):
    print('[%s] %s' % (datetime.now().strftime('%H:%M:%S'), message), flush=True)


def reboot_car(client):
    try:
        password = json.loads(CREDS.read_text())['password']
        client.exec_command('echo %s | sudo -S systemctl reboot' % password, timeout=5)
    except Exception:
        pass


def die(message):
    log('ABORT: ' + message)
    if STATE['localisation_stopped'] and STATE['client'] is not None:
        log('rebooting the car so it does not stay stuck without localisation')
        reboot_car(STATE['client'])
    raise SystemExit(2)


# --------------------------------------------------------------------------
# ssh / car helpers
# --------------------------------------------------------------------------
def ssh_connect(host):
    import paramiko
    password = json.loads(CREDS.read_text())['password']
    client = paramiko.SSHClient()
    try:                                     # prefer a pinned host key
        sys.path.insert(0, str(HERE))
        import ros2_preflight
        return ros2_preflight.connect(host, password=password)
    except Exception:
        pass
    log('note: no pinned host key for %s, trusting it for this session' % host)
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username='pi', password=password, timeout=12,
                   banner_timeout=12, auth_timeout=12)
    return client


def run(client, script, timeout=120, quiet=False):
    """Run a bash snippet on the car without any shell-quoting games."""
    payload = base64.b64encode(script.encode()).decode()
    name = '/tmp/_rebuild_%d.sh' % int(time.time() * 1000)
    command = ('printf %%s %s | base64 -d > %s && bash %s; rc=$?; rm -f %s; exit $rc'
               % (payload, name, name, name))
    _, out, err = client.exec_command(command, timeout=timeout)
    stdout = out.read().decode('utf-8', 'replace')
    stderr = err.read().decode('utf-8', 'replace')
    code = out.channel.recv_exit_status()
    if stderr.strip() and not quiet:
        log('  stderr: ' + stderr.strip().splitlines()[-1][:160])
    return stdout, code


def push_file(client, source, remote):
    data = base64.b64encode(Path(source).read_bytes()).decode()
    run(client, "printf %%s %s | base64 -d > %s" % (data, remote), quiet=True)


def push_and_copy(client, source, name):
    push_file(client, source, '/tmp/' + name)
    run(client, 'docker cp /tmp/%s %s:/tmp/%s' % (name, CORE, name), quiet=True)


# --------------------------------------------------------------------------
# rosbridge helpers
# --------------------------------------------------------------------------
def check_bridge(port=19091, fatal=True):
    from rosbridge_client import Rosbridge
    try:
        bridge = Rosbridge('127.0.0.1', port, timeout=8)
        topics = bridge.topics()
        bridge.close()
        return sorted(topics)
    except Exception as error:
        if not fatal:
            return []
        die('cannot reach rosbridge on 127.0.0.1:%d (%s).\n'
            '       Start both tunnels first:\n'
            '         python tools/ssh_tunnel.py --host %s --local-port 19091 --remote-port 9091\n'
            '         python tools/ssh_tunnel.py --host %s --local-port 19099 --remote-port 8099'
            % (port, error, DEFAULT_HOST, DEFAULT_HOST))


def map_known(bridge):
    message = bridge.read_fresh('/map', 'nav_msgs/msg/OccupancyGrid', timeout=5.0)
    if message is None:
        return None
    data = message['data']
    return {'known': sum(1 for v in data if v >= 0),
            'occ': sum(1 for v in data if v >= 65),
            'shape': '%dx%d' % (message['info']['width'], message['info']['height'])}


# --------------------------------------------------------------------------
# validation
# --------------------------------------------------------------------------
def speckle_ratio(pgm):
    from PIL import Image
    image = Image.open(pgm).convert('L')
    width, height = image.size
    pixels = image.load()
    occupied = [(c, r) for r in range(height) for c in range(width)
                if pixels[c, r] <= 65]
    if not occupied:
        return 1.0, 0
    lonely = 0
    for column, row in occupied:
        neighbours = 0
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if dc == 0 and dr == 0:
                    continue
                nc, nr = column + dc, row + dr
                if 0 <= nc < width and 0 <= nr < height and pixels[nc, nr] <= 65:
                    neighbours += 1
        if neighbours == 0:
            lonely += 1
    return lonely / len(occupied), len(occupied)


def fit_against_live_scan(pgm, span=1.0, step=0.10, yaw_step=4.0):
    """Brute-force fit of the live scan into this map.  Independent of amcl.

    Same search as `scan_map_match.py --map <pgm> --span 1.0 --step 0.1
    --yaw-step 4`: 11x11 positions times 90 headings, about ten seconds here.
    Coarse on purpose -- it has to tell "this map fits the world" from "it does
    not", not to measure millimetres.
    """
    import scan_map_match as sm
    width, height, data = sm.load_map(str(pgm))
    resolution, origin = sm.read_map_yaml(str(pgm))
    dist = sm.distance_transform(width, height, data)
    scan = sm.fetch_scan(19091, 8.0)
    if scan is None:
        die('no /scan from the car; cannot validate the new map')
    points = sm.scan_points(scan, 6.0)
    steps = int(span / step) + 1
    offsets = [(index - steps // 2) * step for index in range(steps)]
    best = None
    for x in offsets:
        for y in offsets:
            for tenth in range(int(360 / yaw_step)):
                yaw_deg = tenth * yaw_step - 180.0
                result = sm.score(points, width, height, dist, resolution, origin,
                                  x, y, math.radians(yaw_deg))
                if result is None:
                    continue
                mean_cells, used = result
                if best is None or mean_cells < best[0]:
                    best = (mean_cells, x, y, yaw_deg, used)
    if best is None:
        return float('inf'), 0.0, 0.0, 0.0, 0, len(points)
    mean_cells, x, y, yaw_deg, used = best
    return mean_cells * resolution, x, y, yaw_deg, used, len(points)


# --------------------------------------------------------------------------
# steps
# --------------------------------------------------------------------------
def survey(client, turns, rate):
    from rosbridge_client import Rosbridge
    import mapping_survey as ms

    bridge = Rosbridge('127.0.0.1', 19091, timeout=12)
    clear, offenders, frames = ms.rotation_is_clear(bridge)
    log('rotation clearance over %d scans: %s'
        % (frames, 'clear' if clear else 'BLOCKED (%d points)' % len(offenders)))
    if not clear:
        for angle, distance in offenders[:5]:
            log('   %6.1f deg  %.3f m' % (angle, distance))
        bridge.close()
        die('refusing to spin: move the obstacle first')

    start = map_known(bridge)
    log('map before the survey: %s' % start)
    delivered = 0.0
    for index in range(1, turns + 1):
        log('revolution %d/%d ...' % (index, turns))
        turned = ms.sa.turn(bridge, 360.0, rate)
        if turned is None:
            bridge.close()
            die('no /odom feedback during the turn')
        delivered += abs(turned)
        log('  delivered %+.1f deg' % turned)
        if abs(turned) < 270:
            log('  less than 270 deg delivered -- stopping rather than repeating')
            break
    after = map_known(bridge)
    bridge.close()
    log('map after the survey : %s' % after)
    if start and after and after['known'] - start['known'] < 500:
        die('the map grew by only %d cells after %.1f deg of commanded rotation.\n'
            '       The wheels slip on this floor: odometry says "turned" while the\n'
            '       robot stands still.  Move the car to a grippier spot and retry.'
            % ((after['known'] - start['known']) if after and start else 0, delivered))
    return delivered


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--attempts', type=int, default=2,
                        help='how many sweeps to try before giving up; a rejected '
                             'sweep leaves visible radial streaks in the map')
    parser.add_argument('--turns', type=int, default=1,
                        help='revolutions to sweep (1 measured enough for amcl)')
    parser.add_argument('--rate', type=float, default=0.25,
                        help='rotation rate, rad/s (0.25 measured reliable)')
    parser.add_argument('--no-install', action='store_true',
                        help='survey and validate only; keep the map already on the car')
    parser.add_argument('--keep-map', action='store_true',
                        help='alias for --no-install')
    parser.add_argument('--yes', action='store_true',
                        help='do not ask for confirmation before moving the car')
    args = parser.parse_args()
    install = not (args.no_install or args.keep_map)

    if not args.yes:
        if not sys.stdin.isatty():
            die('this command rotates the robot; re-run with --yes in a '
                'non-interactive shell')
        answer = input('The robot will spin %d full turn(s) in place. Type yes: ' % args.turns)
        if answer.strip().lower() != 'yes':
            die('cancelled')

    topics = check_bridge()
    log('rosbridge ok, %d topics' % len(topics))

    client = ssh_connect(args.host)
    identity, _ = run(client, 'ls /etc/armpi-ros2/active && docker ps --format "{{.Names}}"', 30)
    if 'armpi-ros2' not in identity and CORE not in identity:
        die('%s does not look like the car (no /etc/armpi-ros2/active)' % args.host)
    log('car identity confirmed on %s' % args.host)

    stamp = datetime.now().strftime('%Y%m%d_%H%M')
    remote_prefix = '/tmp/rebuild_%s' % stamp
    window = int(30 * args.turns + 45)

    STATE['client'] = client
    log('stopping the localisation stack (map_server would fight over /map)')
    run(client, 'sh /home/pi/localization/run.sh stop 2>&1 | tail -1 || true', 60)
    STATE['localisation_stopped'] = True
    time.sleep(3)

    log('uploading the mapping helpers into the container')
    for source, name in UPLOADS:
        push_and_copy(client, ROOT / source, name)
    listing, _ = run(client, 'docker exec %s bash -c "ls /tmp/*.py /tmp/*.sh /tmp/*.yaml"'
                     % CORE, 30, quiet=True)
    for _, name in UPLOADS:
        if name not in listing:
            die('%s did not land in the container' % name)

    local_pgm = ROOT / 'artifacts' / ('rebuild_%s.pgm' % stamp)
    local_yaml = local_pgm.with_suffix('.yaml')
    accepted = False
    for attempt in range(1, args.attempts + 1):
        if args.attempts > 1:
            log('=== sweep attempt %d/%d ===' % (attempt, args.attempts))
        run(client, 'docker exec %s bash -c "pkill -f \'[s]lam_session.sh\'; '
                    'pkill -f \'[w]atch_map_growth\'; pkill -9 -f async_slam_toolbox_node; '
                    'rm -f %s.pgm %s.yaml" || true'
                    % (CORE, remote_prefix, remote_prefix), 30, quiet=True)
        time.sleep(2)

        log('starting slam_toolbox (window %d s)' % window)
        run(client, 'docker exec -d %s bash -c "bash /tmp/slam_session.sh %d %s 6.0 '
                    '> /tmp/session_console.log 2>&1"' % (CORE, window, remote_prefix), 60)

        ready = False
        for _ in range(30):
            time.sleep(1)
            state, _ = run(client, 'docker exec %s bash -lc "source /opt/ros/jazzy/setup.bash '
                                   '>/dev/null 2>&1; timeout 4 ros2 lifecycle get /slam_toolbox '
                                   '2>/dev/null | tail -1"' % CORE, 30, quiet=True)
            if 'active' in state:
                ready = True
                break
        if not ready:
            tail, _ = run(client, 'docker exec %s tail -6 /tmp/session_console.log' % CORE, 30)
            die('slam_toolbox never became active://n' + tail)
        log('slam_toolbox active')

        delivered = survey(client, args.turns, args.rate)
        log('survey finished: %.1f deg commanded, car has NOT translated' % delivered)

        log('waiting for the map to be saved')
        saved = False
        for _ in range(90):
            time.sleep(2)
            check, _ = run(client, 'docker exec %s test -f %s.pgm && echo yes'
                           % (CORE, remote_prefix), 30, quiet=True)
            if 'yes' in check:
                saved = True
                break
        if not saved:
            die('the map was never written to %s.pgm' % remote_prefix)

        log('pulling the map to %s' % local_pgm.relative_to(ROOT))
        for remote, target in (('%s.pgm' % remote_prefix, local_pgm),
                               ('%s.yaml' % remote_prefix, local_yaml)):
            blob, _ = run(client, 'docker exec %s base64 -w0 %s' % (CORE, remote), 60,
                          quiet=True)
            target.write_bytes(base64.b64decode(blob))
            if target.stat().st_size < 100:
                die('%s came back empty' % remote)

        speckle, occupied = speckle_ratio(local_pgm)
        log('quality: %d occupied cells, speckle %.1f%%' % (occupied, speckle * 100))
        mean, x, y, yaw, used, total = fit_against_live_scan(local_pgm)
        log('independent fit: %.3f m mean, pose (%+.2f, %+.2f, %+.1f deg), %d/%d beams'
            % (mean, x, y, yaw, used, total))

        problems = []
        if occupied < MIN_OCCUPIED:
            problems.append('only %d occupied cells' % occupied)
        if speckle > MAX_SPECKLE:
            problems.append('speckle %.1f%% > %.0f%%' % (speckle * 100, MAX_SPECKLE * 100))
        if mean > MAX_FIT_M:
            problems.append('scan sits %.3f m off the walls (limit %.2f m)'
                            % (mean, MAX_FIT_M))
        if not problems:
            accepted = True
            fit_pose = (x, y, yaw)
            log('validation PASSED')
            break

        # A rejected sweep needs a NEW mapper process: the bad pose graph lives in
        # the old one, so rotating again with it still running just adds more
        # streaks.  Measured 2026-11-10: run 2 came out at 6.2% speckle with
        # visible radial smears while run 1 of the same room was 1.3% and clean.
        log('attempt %d REJECTED: %s' % (attempt, '; '.join(problems)))
        run(client, 'docker exec %s bash -c "pkill -f \'[s]lam_session.sh\'; '
                    'pkill -f \'[w]atch_map_growth\'; pkill -9 -f async_slam_toolbox_node" '
                    '|| true' % CORE, 30, quiet=True)
        if attempt == args.attempts:
            die('no sweep produced a usable map. The car is fine; the room may be '
                'too cluttered or too reflective for this lidar from this spot.')


    if not install:
        log('--no-install: leaving the map on the car untouched')
        install_ok = True
    else:
        today = datetime.now().strftime('%Y%m%d')
        log('installing as workshop.* (current map backed up to workshop_%s.*)' % today)
        # Both the sweep output and the map directory live INSIDE the core
        # container.  Doing `cp /tmp/rebuild_*.pgm /opt/armpi-maps/...` from the
        # host fails with "No such file or directory" -- twice, 2026-11-10.
        script = ('docker exec %s bash -c \'set -e; cd %s; '
                  'cp -f workshop.pgm workshop_%s.pgm; '
                  'cp -f workshop.yaml workshop_%s.yaml; '
                  'cp /tmp/rebuild_%s.pgm workshop.pgm\''
                  % (CORE, MAPS_DIR, today, today, stamp))
        out, code = run(client, script, 60)
        yaml_text = local_yaml.read_text().replace(
            'image: rebuild_%s.pgm' % stamp, 'image: workshop.pgm')
        local_yaml.write_text(yaml_text)
        if code == 0:
            push_file(client, local_yaml, '/tmp/workshop_new.yaml')
            run(client, 'docker cp /tmp/workshop_new.yaml %s:/tmp/workshop_new.yaml' % CORE, 30)
            _, code = run(client, 'docker exec %s cp /tmp/workshop_new.yaml %s/workshop.yaml'
                          % (CORE, MAPS_DIR), 30)
        install_ok = code == 0

        if install_ok:
            verify, _ = run(client, 'docker exec %s bash -c "head -2 %s/workshop.pgm; '
                                    'grep ^image %s/workshop.yaml"' % (CORE, MAPS_DIR, MAPS_DIR), 30)
            log('installed: ' + ' | '.join(verify.split('\n')[:3]))
            repo_pgm = ROOT / 'maps' / ('workshop_%s.pgm' % today)
            repo_yaml = repo_pgm.with_suffix('.yaml')
            repo_pgm.write_bytes(local_pgm.read_bytes())
            repo_yaml.write_text(yaml_text.replace(
                'image: workshop.pgm', 'image: workshop_%s.pgm' % today))
            log('archived in the repo at %s' % repo_pgm.relative_to(ROOT))

            # A stale pose belongs to the old map.  The keeper tags poses with a
            # map fingerprint so a mismatch gets ignored, but a clean start lets
            # the first boot seed broadly and then remember again from here.
            # remove it from inside the container: that runs as root and sees
            # the same bind mount, no sudo prompt on the host
            run(client, 'docker exec %s rm -f %s/last_pose.json' % (CORE, MAPS_DIR), 30)
        else:
            log('WARNING: install failed (%s) -- the map on the car is unchanged' % out.strip()[:160])

    # The only reliable way back to a working localisation stack: a full reboot.
    # Restarting just armpi-localization has twice left amcl with a frozen TF
    # listener that never publishes again.
    log('rebooting the car (a sidecar-only restart can freeze amcl)')
    reboot_car(client)
    STATE['localisation_stopped'] = False
    client.close()

    log('waiting for the four units (up to 5 min)')
    deadline = time.time() + 420
    state = ''
    while time.time() < deadline:
        time.sleep(8)
        try:
            client = ssh_connect(args.host)
        except Exception:
            continue
        state, _ = run(client, 'systemctl is-active armpi-ros2 armpi-tf-publisher '
                               'armpi-frame-relay armpi-localization | tr "\\n" " "',
                       30, quiet=True)
        state = ' '.join(state.split())
        log('  boot: ' + (state or 'no answer yet'))
        if state == 'active active active active':
            break
        client.close()
    else:
        die('the car did not come back cleanly; last state: %s' % state)
    log('all four units active')

    seed, _ = run(client, 'tail -12 /run/armpi-ros2/localization-seed.log', 30, quiet=True)
    print(seed.strip())

    # The first boot on a NEW map has no remembered pose, so amcl has to search
    # globally -- which on 2026-11-09/10 succeeded roughly one time in three.
    # But we do know where the car is: the validation fit measured its pose in
    # this very map.  Seed from that, tightly, instead of leaving the robot
    # unlocalised until someone does it by hand.
    if 'CONVERGED' not in seed.upper():
        fit_x, fit_y, fit_yaw = fit_pose
        log('boot seed did not converge; seeding from the validated pose '
            '(%+.2f, %+.2f, %+.1f deg)' % (fit_x, fit_y, fit_yaw))
        bootstrap, _ = run(
            client,
            'docker exec %s bash -lc "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && '
            'python3 %s/amcl_global_seed.py --x %.3f --y %.3f --yaw %.4f '
            '--sigma-xy 0.15 --sigma-yaw 0.10 --wait 15"'
            % (CORE, MAPS_DIR, fit_x, fit_y, math.radians(fit_yaw)), 120)
        print(bootstrap.strip())
        if 'CONVERGED' not in bootstrap.upper():
            log('WARNING: still not converged. The map is installed and correct; '
                'localisation needs the car to move (a slow 360 deg turn).')
    client.close()

    log('restarting the two tunnels')
    env = dict(os.environ)
    env['ARMPI_SSH_PASSWORD'] = json.loads(CREDS.read_text())['password']
    flags = 0
    if os.name == 'nt':
        flags = 0x00000008 | 0x00000200          # DETACHED_PROCESS | NEW_PROCESS_GROUP
    for local, remote in ((19091, 9091), (19099, 8099)):
        subprocess.Popen([sys.executable, str(TUNNEL_SCRIPT), '--host', args.host,
                          '--local-port', str(local), '--remote-port', str(remote)],
                         env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         creationflags=flags)
    time.sleep(4)
    topics = []
    for _ in range(12):
        topics = check_bridge(fatal=False)
        if topics:
            break
        time.sleep(2)
    if not topics:
        die('the tunnels did not come back; start them by hand (see above)')
    log('tunnels back, %d topics visible' % len(topics))
    log('note: those tunnel processes belong to this shell -- if you close it, '
        'restart them with tools/ssh_tunnel.py (both ports)')
    if not install_ok:
        log('finished, but the NEW MAP WAS NOT INSTALLED -- see the warning above')
        return 2
    log('DONE. Verify with: python tools/scan_map_match.py --span 0.6 --step 0.05')
    return 0


if __name__ == '__main__':
    sys.exit(main())
