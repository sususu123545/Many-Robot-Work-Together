#!/usr/bin/env python3
"""Drive a SMALL, conservative mapping survey: spin in place, do not roam.

Why spinning is enough
----------------------
The LD06 is a 360-degree 2D lidar and slam_toolbox builds an occupancy grid.
A full rotation already sweeps everything around the car, so no long traverse
is needed.  `minimum_travel_heading = 0.20 rad` (~11.5 deg) in
slam_toolbox_mapping.yaml means one revolution inserts 30+ scans.

This is also the safe choice on this chassis: rotary odometry measured accurate
(90.3 / 90.6 / 90.5 deg for three commanded 90 deg turns) while LINEAR travel
shows ~38% error and the mecanum wheels slip on smooth floor.  Spinning plays
to the car's strength and avoids its weakness.

Safety (all measured the hard way on this car)
---------------------------------------------
* `search_around.turn()` owns its own /cmd_vel and /odom connections -- one
  connection doing both subscribe and publish delivered only 29% of commands.
* It installs SIGINT/SIGTERM handlers that send a stop FIRST.  The chassis has
  NO /cmd_vel deadman: the last command it heard is the one it keeps obeying.
* It has a 4-second no-progress stall guard.
* Before any motion this script checks the rotation clearance over 3 scans.

Usage (slam_toolbox must already be running):
    python tools/mapping_survey.py --turns 2
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'perception'))

from rosbridge_client import Rosbridge                          # noqa: E402
import scan_clean                                              # noqa: E402
import search_around as sa                                     # noqa: E402


def log(message):
    print(message, flush=True)


def rotation_is_clear(bridge):
    bridge.subscribe('/scan', 'sensor_msgs/msg/LaserScan')
    bridge.drain(0.6)
    scans = []
    for _ in range(3):
        message = bridge.wait_for_topic('/scan', timeout=4)
        if message is not None:
            scans.append(scan_clean.clean_scan(
                message['ranges'], message['angle_increment']))
        time.sleep(0.2)
    if not scans:
        return False, [], 0
    clear, offenders = scan_clean.rotation_clearance_agreed(scans)
    return clear, offenders, len(scans)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--turns', type=int, default=2,
                        help='full revolutions in place (2 is usually enough)')
    parser.add_argument('--rate', type=float, default=0.25,
                        help='rotation rate rad/s; 0.25 measured reliable')
    parser.add_argument('--max-laser-range', type=float, default=3.0,
                        help='informational: pass the SAME value to the '
                             'slam_toolbox launch so the map stays small')
    parser.add_argument('--force', action='store_true',
                        help='skip the rotation-clearance precheck (not advised)')
    args = parser.parse_args()

    if args.turns < 1 or args.turns > 6:
        log('refusing: --turns must be 1..6')
        return 2

    bridge = Rosbridge('127.0.0.1', 19091, timeout=12)
    if not args.force:
        clear, offenders, frames = rotation_is_clear(bridge)
        log('rotation clearance over %d scans: %s'
            % (frames, 'clear' if clear else 'BLOCKED (%d points)' % len(offenders)))
        if not clear:
            for angle, distance in offenders[:6]:
                log('   %6.1f deg  %.3f m' % (angle, distance))
            log('refusing to spin. Move the obstacle or check the lidar.')
            return 3

    log('mapping patch: laser clipped to %.1f m (pass this to the launch too)'
        % args.max_laser_range)
    total = 0.0
    for index in range(1, args.turns + 1):
        log('--- revolution %d/%d ---' % (index, args.turns))
        delivered = sa.turn(bridge, 360.0, args.rate)
        if delivered is None:
            log('no /odom feedback -- stopping the survey')
            return 4
        total += abs(delivered)
        log('  delivered %+.1f deg' % delivered)
        if abs(delivered) < 270:
            log('  less than 270 deg delivered -- stopping rather than '
                'repeating a turn that is not happening')
            break
        time.sleep(1.0)

    log('survey done: %.1f deg of rotation, car has NOT translated' % total)
    log('next: save /map with tools/save_occupancy_map.py, then stop slam_toolbox')
    return 0


if __name__ == '__main__':
    sys.exit(main())
