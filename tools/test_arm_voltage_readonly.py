import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from armpi_ros2 import servo_protocol

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


with patch.dict(sys.modules, {'servo_protocol': servo_protocol}):
    ab = load('ab_voltage_test', 'tools/arm_transport_readonly_ab.py')
    with patch.dict(sys.modules, {'arm_transport_readonly_ab': ab}):
        voltage = load('voltage_probe_test', 'tools/arm_voltage_readonly.py')


class VoltageTests(unittest.TestCase):
    def test_read_plan_contains_only_exact_physical_read_formats(self):
        self.assertEqual([c for _, c, _ in voltage.READS], [5, 7, 9])
        for _, command, fmt in voltage.READS:
            self.assertEqual(servo_protocol.READ_FORMATS[command], fmt)
            ab.check_read_packet(servo_protocol.packet(bytes([command, 1])))

    def test_summary_does_not_invent_missing_measurements(self):
        rows = [{'servo': 1, 'millivolts': 7860, 'position': 100, 'temperature': 29},
                {'servo': 1, 'position': 101, 'temperature': 30},
                {'servo': 1, 'millivolts': 7840}]
        result = voltage.summarize(rows)
        self.assertEqual(result[1]['millivolts'], {'samples': 2, 'min': 7840, 'max': 7860, 'latest': 7840})
        self.assertEqual(result[1]['temperature']['samples'], 2)
        self.assertEqual(result[2]['millivolts'], {'samples': 0})


if __name__ == '__main__': unittest.main()
