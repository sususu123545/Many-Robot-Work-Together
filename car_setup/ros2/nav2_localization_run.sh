#!/bin/sh
# Bring up occupancy-map localisation at boot: map_server + amcl + a global seed.
#
# Why exec into the core container instead of running a sidecar container
# ----------------------------------------------------------------------
# arm_tf_publisher and frame_relay get their own containers because they ship
# their own files and need `--ipc container:<core>`.  The localisation stack is
# different: its code is already in the image, its map is already bind-mounted
# at /opt/armpi-maps, and it needs the core's ROS_DOMAIN_ID / discovery env,
# which a fresh container would NOT inherit (the core gets those from `-e` on
# `docker run`, not from the image).  Running `docker exec` inherits all of it.
#
# This does NOT reintroduce the "one dead node kills the whole graph" problem:
# that applies to nodes inside the core's own launch, where PID 1 exits and the
# unit goes down.  A `docker exec` process is a separate child of the container;
# if it dies, PID 1 never notices.
#
# When the core is restarted the container is recreated and every exec'd process
# dies with it, so there is nothing to clean up on the stop side.
set -eu

CORE=armpi_ros2_native
CFG=/opt/armpi-ros2-localization
# This script runs on the HOST, but the paths it hands to `docker exec` are the
# container's.  Keep both: the existence check must use the host side, or it
# fails with "missing ... -- is it mounted?" while the mount is perfectly fine.
HOST_MAPS=/var/lib/armpi-ros2/maps
PARAMS_HOST=$HOST_MAPS/nav2_localization_params.yaml
PARAMS=/opt/armpi-maps/nav2_localization_params.yaml
RUNDIR=/run/armpi-ros2
# Container path, not a host path: the redirect runs INSIDE the container.
LOG=/tmp/localization.log
SEEDLOG=$RUNDIR/localization-seed.log
STATUS=$RUNDIR/localization.json
WAIT_SECONDS=${WAIT_SECONDS:-180}

mkdir -p "$RUNDIR"

# `run.sh stop` is what the unit's ExecStop calls.  Keeping the cleanup here
# instead of in the unit avoids nested-quoting between systemd and sh, which is
# a reliable source of silently-misparsed ExecStop lines.
cleanup_nodes() {
    docker exec "$CORE" bash -c \
        'pkill -9 -x amcl; pkill -9 -x map_server; pkill -9 -x lifecycle_manag; pkill -9 -f "[n]av2_localization.launch.py"' >/dev/null 2>&1 || true
}

if [ "${1:-start}" = "stop" ]; then
    cleanup_nodes
    echo "localisation nodes stopped"
    exit 0
fi

if [ ! -f "$PARAMS_HOST" ]; then
    echo "missing $PARAMS_HOST -- is /var/lib/armpi-ros2/maps present?" >&2
    exit 1
fi

# The core is recreated on every restart; wait for the NEW one before exec'ing.
waited=0
while [ "$waited" -lt "$WAIT_SECONDS" ]; do
    if docker inspect "$CORE" --format '{{.State.Running}}' 2>/dev/null | grep -q true; then
        break
    fi
    sleep 3
    waited=$((waited + 3))
done
if ! docker inspect "$CORE" --format '{{.State.Running}}' 2>/dev/null | grep -q true; then
    echo "core container $CORE did not come up within ${WAIT_SECONDS}s" >&2
    exit 1
fi
echo "core=$CORE up after ${waited}s"

# A previous generation of this unit may have left same-named nodes behind.  Two
# sets of /amcl and /map_server fight over /initialpose and /amcl_pose and
# neither converges (measured 2026-11-07).  `ps` truncates comm to 15 chars, so
# lifecycle_manager has to be matched as lifecycle_manag.
# The `ros2 launch` wrapper survives its children, so it has to be matched by
# its own command line.  Never `pkill -x ros2`: the core's own launch process has
# that exact comm, and killing it would take the whole robot down.  The `[n]`
# keeps the pattern from matching the killer's own command line.
cleanup_nodes
sleep 2
# `grep -c` prints 0 AND exits 1 when there is no match, so `|| echo 0` would
# append a second line and the string comparison below would never see "0".
stale=$(docker exec "$CORE" bash -c \
    'ps -eo comm= | grep -cxE "amcl|map_server|lifecycle_manag" || true' 2>/dev/null | head -1)
stale=${stale:-0}
if [ "$stale" != "0" ]; then
    echo "REFUSING: $stale stale localisation node(s) in the core container." >&2
    exit 1
fi

docker exec -d "$CORE" bash -lc \
    "source /opt/ros/jazzy/setup.bash && exec ros2 launch $CFG/nav2_localization.launch.py params_file:=$PARAMS >$LOG 2>&1"
echo "launch started, waiting for amcl to become active"

state=""
i=0
while [ "$i" -lt 20 ]; do
    state=$(docker exec "$CORE" bash -lc \
        'source /opt/ros/jazzy/setup.bash; ros2 lifecycle get /amcl 2>/dev/null' 2>/dev/null || true)
    case "$state" in
        active*) break ;;
    esac
    sleep 2
    i=$((i + 1))
done
echo "amcl state: ${state:-unknown}"
case "$state" in
    active*) ;;
    *) echo "amcl did not become active; container log:" >&2
       docker exec "$CORE" tail -20 "$LOG" >&2 || true
       exit 1 ;;
esac

# Global localisation needs an external broad /initialpose -- nav2_amcl never
# seeds itself and refuses to publish any pose until it has been given one.
seed_rc=0
docker exec "$CORE" bash -lc \
    "source /opt/ros/jazzy/setup.bash && python3 /opt/armpi-maps/amcl_global_seed.py --wait 20" \
    >"$SEEDLOG" 2>&1 || seed_rc=$?
tail -6 "$SEEDLOG" || true

verdict=$(grep -o 'verdict *: .*' "$SEEDLOG" 2>/dev/null | head -1 | sed 's/verdict *: //' || true)
case "$seed_rc" in
    0) converged=true ;;
    *) converged=false ;;
esac
printf '{"converged": %s, "amcl_state": "%s", "verdict": "%s", "updated": "%s"}\n' \
    "$converged" "$state" "${verdict:-none}" "$(date -Iseconds)" >"$STATUS"
echo "status: $(cat "$STATUS")"

if [ "$converged" != "true" ]; then
    # Not an error: the car may simply be standing still in a symmetric place.
    # But nobody should trust /amcl_pose in this state, so say it loudly.
    echo "WARNING: localisation has NOT converged. /amcl_pose is not trustworthy" >&2
    echo "         until the car moves (a slow 360 deg turn usually fixes it)." >&2
fi

docker ps --filter "name=$CORE" --format '{{.Names}} {{.Status}}'
exit 0
