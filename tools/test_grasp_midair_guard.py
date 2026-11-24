"""Regression: the mid-air close guard must cover the first close too.

2026-11-24 real run: the descent tripped the floor guard, the script lifted
the arm to clear the surface (~93 mm above the plane), and the first close
then ran at that lifted height -- ~79 mm above the located ball centre.  The
jaws stalled at 452 on nothing while the operator watched them close well
above the ball.  The guard existed only inside the nudge branch; the "as is"
close had none.  These cases use the actual servo ticks from that run.
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src',
                                'armpi_ros2', 'armpi_ros2'))

import grasp_sequence as gs                 # noqa: E402
from arm_kinematics import forward_tool_pose_raw   # noqa: E402

#: Ball centre as the ground estimator located it during that run.
BALL_Z = -0.080

#: Servo ticks at the moment of the empty close (after the floor-abort lift).
LIFTED_TICKS = {'joint1': 475, 'joint2': 434, 'joint3': 921, 'joint4': 415}

#: Servo ticks where the descent actually stopped against the guard, before
#: the lift.  The FK of this pose matches the run's own "+11 mm against the
#: aim" log line, so it is a trustworthy reference.
STOPPED_TICKS = {'joint1': 475, 'joint2': 367, 'joint3': 921, 'joint4': 415}


def tool_z(ticks):
    return forward_tool_pose_raw(ticks)['translation_m'][2]


class TestMidAirCloseGuard(unittest.TestCase):

    def test_the_2026_11_24_lifted_close_is_blocked(self):
        height = tool_z(LIFTED_TICKS) - BALL_Z
        self.assertGreater(height, 0.045)
        self.assertTrue(gs.close_blocked_midair(tool_z(LIFTED_TICKS), BALL_Z))

    def test_the_guard_stop_height_may_still_try_to_close(self):
        # The descent stopped 0.9 mm above the ball top there; a close at
        # that height is the whole point of the dz ladder, so it must pass.
        height = tool_z(STOPPED_TICKS) - BALL_Z
        self.assertLess(height, 0.045)
        self.assertFalse(gs.close_blocked_midair(tool_z(STOPPED_TICKS), BALL_Z))

    def test_a_normally_descended_level_is_not_blocked(self):
        normal = BALL_Z + 0.008          # the dz=+8 aim
        self.assertFalse(gs.close_blocked_midair(normal, BALL_Z))

    def test_limit_matches_the_documented_constant(self):
        self.assertEqual(gs.MIDAIR_CLOSE_LIMIT_M, 0.045)

    def test_ball_below_the_tool_is_the_only_direction_that_blocks(self):
        # A tool point BELOW the ball centre is a normal deep dz, never a
        # reason to refuse -- the guard is one-sided on purpose.
        self.assertFalse(gs.close_blocked_midair(BALL_Z - 0.01, BALL_Z))


if __name__ == '__main__':
    unittest.main()
