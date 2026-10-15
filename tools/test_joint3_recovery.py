import contextlib
import importlib.util
import io
from pathlib import Path
from unittest import TestCase, main, mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('recovery', ROOT / 'tools/joint3_recover.py')
RECOVERY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RECOVERY)


class Board:
    def __init__(self, values, temperature=25):
        self.values = iter(values)
        self.temp = temperature
        self.moves = []
        self.closed = False

    def limits(self, ident):
        return [0, 1000]

    def position(self, ident):
        return next(self.values)

    def temperature(self, ident):
        return self.temp

    def move(self, duration, targets):
        self.moves.append((duration, targets))

    def close(self):
        self.closed = True


class RecoveryTests(TestCase):
    def run_case(self, values, temperature=25, **settings):
        board = Board(values, temperature)
        defaults = dict(MODE='run', CONFIRM='AREA_CLEAR', SERVO=4,
                        FINAL=907, STEP=25, DURATION=1.5, SETTLE=1.8,
                        MESSAGE_CAP=12, TEMP_ABORT=55)
        defaults.update(settings)
        with mock.patch.object(RECOVERY, 'ServoBoard', return_value=board), \
                mock.patch.multiple(RECOVERY, **defaults), \
                mock.patch.object(RECOVERY.time, 'sleep'), \
                contextlib.redirect_stdout(io.StringIO()):
            result = RECOVERY.main()
        return result, board

    def test_inward_only_bounded_recovery(self):
        code, board = self.run_case([1002, 1002, 984, 984, 966, 966, 948, 948, 930, 930, 915])
        self.assertEqual(code, 0)
        self.assertEqual(board.moves, [(1.5, [(4, n)]) for n in [977,959,941,923,907]])
        self.assertTrue(board.closed)

    def test_no_first_move_if_hot(self):
        code, board = self.run_case([1025], temperature=55)
        self.assertEqual(code, 2)
        self.assertEqual(board.moves, [])

    def test_unknown_position_refused(self):
        code, board = self.run_case([1041])
        self.assertEqual(code, 2)
        self.assertEqual(board.moves, [])

    def test_drift_before_command_refused(self):
        code, board = self.run_case([1025, 1030])
        self.assertEqual(code, 3)
        self.assertEqual(board.moves, [])

    def test_wrong_direction_aborts_immediately(self):
        code, board = self.run_case([1025, 1025, 1028])
        self.assertEqual(code, 3)
        self.assertEqual(len(board.moves), 1)

    def test_typo_mode_never_moves(self):
        code, board = self.run_case([], MODE='RUN')
        self.assertEqual(code, 2)
        self.assertEqual(board.moves, [])

    def test_confirmation_required(self):
        code, board = self.run_case([], CONFIRM='')
        self.assertEqual(code, 2)
        self.assertEqual(board.moves, [])

    def test_plan_never_moves(self):
        code, board = self.run_case([1025], MODE='plan')
        self.assertEqual(board.moves, [])

    def test_guard_cannot_be_relaxed(self):
        for change in ({'STEP':26}, {'DURATION':.6}, {'SERVO':1}, {'TEMP_ABORT':65}):
            code, board = self.run_case([], **change)
            self.assertEqual(code, 2)
            self.assertEqual(board.moves, [])

    def test_legal_but_near_limit_is_not_mistaken_for_safe_margin(self):
        code, board = self.run_case([998,998,980], MESSAGE_CAP=1)
        self.assertEqual(code,3)
        self.assertEqual(board.moves,[(1.5,[(4,973)])])

    def test_normal_pose_is_not_touched(self):
        code, board = self.run_case([907])
        self.assertEqual(code,0)
        self.assertEqual(board.moves,[])


if __name__ == '__main__':
    main()
