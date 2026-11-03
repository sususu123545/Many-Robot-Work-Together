from __future__ import annotations

import unittest

import numpy as np

from g1_mission_controller import MissionController, MissionState
from g1_task_perception import RGBDColorDetector
from g1_task_stack import G1TaskStack


def det(label="red", kind="wire_candidate", x=1.0, stamp=1.0, frame=1, confidence=.95):
    return {"label": label, "kind": kind, "confidence": confidence,
            "position_base": (x, 0.0, .2), "timestamp": stamp,
            "frame_id": frame}


def healthy(age=.01):
    return {"healthy": True, "sensor_age_s": age, "fallen": False, "e_stop": False}


class MissionControllerTests(unittest.TestCase):
    def setUp(self):
        self.config = {"wire_color_sequence": ["red", "yellow"],
                       "medical_supply_order": ["gauze"],
                       "perception": {"stable_frames": 2}}

    def test_missing_official_order_never_moves(self):
        controller = MissionController(None)
        out = controller.tick(0, [det()], healthy())
        self.assertEqual(out["action"], "hold")
        self.assertEqual(controller.state, MissionState.WAIT_CONFIG)

    def test_wrong_color_decoy_and_unstable_detection_do_not_advance(self):
        controller = MissionController(self.config)
        out = controller.tick(1., [det("yellow", "wire_candidate")], healthy())
        self.assertEqual(out["action"], "hold")
        out = controller.tick(1.1, [det("red", "decoy", frame=2, stamp=1.1)], healthy())
        self.assertEqual(out["action"], "hold")
        out = controller.tick(1.2, [det("red", frame=3, stamp=1.2)], healthy())
        self.assertEqual(out["action"], "hold")
        self.assertEqual(controller.wire_index, 0)
        out = controller.tick(1.3, [det("red", frame=4, stamp=1.3)], healthy())
        self.assertEqual(out["action"], "approach_wire")

    def test_unverified_wire_result_stops_without_advancing(self):
        controller = MissionController(self.config)
        out = controller.tick(1., [det(frame=1, stamp=1.)], healthy())
        out = controller.tick(1.1, [det(frame=2, stamp=1.1)], healthy())
        approach_id = out["command_id"]
        out = controller.tick(1.2, [], healthy(), {"command_id": approach_id, "status": "succeeded"})
        self.assertEqual(controller.state, MissionState.REMOVE_WIRE)
        out = controller.tick(1.3, [det(frame=3, stamp=1.3)], healthy())
        out = controller.tick(1.4, [det(frame=4, stamp=1.4)], healthy())
        remove_id = out["command_id"]
        out = controller.tick(1.5, [], healthy(),
                              {"command_id": remove_id, "status": "succeeded", "verified": False})
        self.assertEqual(out["action"], "halt")
        self.assertEqual(controller.wire_index, 0)
        self.assertEqual(controller.state, MissionState.SAFE_STOP)

    def test_verified_wire_removal_advances_to_next_required_color(self):
        controller = MissionController(self.config)
        controller.tick(1., [det(frame=1, stamp=1.)], healthy())
        out = controller.tick(1.1, [det(frame=2, stamp=1.1)], healthy())
        out = controller.tick(1.2, [], healthy(),
                              {"command_id": out["command_id"], "status": "succeeded"})
        controller.tick(1.3, [det(frame=3, stamp=1.3)], healthy())
        out = controller.tick(1.4, [det(frame=4, stamp=1.4)], healthy())
        out = controller.tick(1.5, [], healthy(),
                              {"command_id": out["command_id"], "status": "succeeded", "verified": True})
        self.assertEqual(controller.wire_index, 1)
        self.assertEqual(controller.state, MissionState.FIND_WIRE)

    def test_repeated_color_skips_already_verified_target_track(self):
        config = {"wire_color_sequence": ["red", "red"], "medical_supply_order": ["gauze"],
                  "perception": {"stable_frames": 1}}
        controller = MissionController(config)
        first = det("red", frame=1, stamp=1.)
        first["track_id"] = "wire-a"
        out = controller.tick(1., [first], healthy())
        out = controller.tick(1.1, [], healthy(),
                              {"command_id": out["command_id"], "status": "succeeded"})
        first = det("red", frame=2, stamp=1.2)
        first["track_id"] = "wire-a"
        out = controller.tick(1.2, [first], healthy())
        out = controller.tick(1.3, [], healthy(),
                              {"command_id": out["command_id"], "status": "succeeded", "verified": True})
        self.assertIn("wire-a", controller.completed_wire_target_ids)
        first = det("red", frame=3, stamp=1.4)
        first["track_id"] = "wire-a"
        out = controller.tick(1.4, [first], healthy())
        self.assertEqual(out["action"], "hold")
        second = det("red", frame=4, stamp=1.5, x=.8)
        second["track_id"] = "wire-b"
        out = controller.tick(1.5, [first, second], healthy())
        self.assertEqual(out["action"], "approach_wire")
        self.assertEqual(out["target_id"], "wire-b")

    def test_supply_must_be_picked_and_verified_in_kit_before_completion(self):
        config = {"wire_color_sequence": ["red"], "medical_supply_order": ["gauze"],
                  "perception": {"stable_frames": 1}}
        controller = MissionController(config)
        out = controller.tick(1., [det(frame=1, stamp=1.)], healthy())
        out = controller.tick(1.1, [], healthy(),
                              {"command_id": out["command_id"], "status": "succeeded"})
        out = controller.tick(1.2, [det(frame=2, stamp=1.2)], healthy())
        out = controller.tick(1.3, [], healthy(),
                              {"command_id": out["command_id"], "status": "succeeded", "verified": True})
        self.assertEqual(controller.state, MissionState.FIND_SUPPLY)

        gauze = det("gauze", "supply_candidate", x=.8, stamp=1.4, frame=3)
        out = controller.tick(1.4, [gauze], healthy())
        self.assertEqual(out["action"], "approach_supply")
        out = controller.tick(1.5, [], healthy(),
                              {"command_id": out["command_id"], "status": "succeeded"})
        gauze = det("gauze", "supply_candidate", x=.7, stamp=1.6, frame=4)
        out = controller.tick(1.6, [gauze], healthy())
        self.assertEqual(out["action"], "pick_supply")
        out = controller.tick(1.7, [], healthy(),
                              {"command_id": out["command_id"], "status": "succeeded", "verified": True})
        self.assertEqual(out["reason"], "searching_for_kit")
        kit = det("kit", "kit_candidate", x=1.0, stamp=1.8, frame=5)
        out = controller.tick(1.8, [kit], healthy())
        self.assertEqual(out["action"], "place_supply")
        out = controller.tick(1.9, [], healthy(),
                              {"command_id": out["command_id"], "status": "succeeded", "verified": True})
        self.assertEqual(controller.state, MissionState.COMPLETE)
        self.assertEqual(out["action"], "halt")

    def test_action_timeout_stops_without_duplicate_manipulation(self):
        config = {"wire_color_sequence": ["red"], "medical_supply_order": ["gauze"],
                  "perception": {"stable_frames": 1},
                  "control": {"action_timeout_s": .5}}
        controller = MissionController(config)
        out = controller.tick(1., [det(frame=1, stamp=1.)], healthy())
        controller.tick(1.1, [det(frame=2, stamp=1.1)], healthy())
        out = controller.tick(1.6, [], healthy())
        self.assertEqual(out["action"], "halt")
        self.assertEqual(controller.state, MissionState.SAFE_STOP)

    def test_sensor_staleness_latches_safe_stop(self):
        controller = MissionController(self.config)
        out = controller.tick(1., [], healthy(age=3.0))
        self.assertEqual(out["action"], "halt")
        self.assertEqual(controller.state, MissionState.SAFE_STOP)
        out = controller.tick(1.1, [det(frame=2, stamp=1.1)], healthy())
        self.assertEqual(out["action"], "halt")


class RGBDDetectorTests(unittest.TestCase):
    def test_red_and_yellow_blobs_project_from_rgbd_without_scene_labels(self):
        rgb = np.zeros((40, 60, 3), dtype=np.uint8)
        rgb[:, :] = (35, 35, 35)
        rgb[10:20, 8:18] = (230, 25, 20)
        rgb[10:20, 38:48] = (240, 200, 20)
        depth = np.full((40, 60), 2.0, dtype=np.float32)
        k = np.array([[50., 0., 30.], [0., 50., 20.], [0., 0., 1.]])
        found = RGBDColorDetector(min_area_px=8).detect(
            {"rgb": rgb, "depth": depth, "K": k, "seconds": .1}, frame_id=7)
        self.assertEqual({d.label for d in found}, {"red", "yellow"})
        self.assertTrue(all(d.kind == "wire_candidate" and d.frame_id == 7 for d in found))
        again = RGBDColorDetector(min_area_px=8)
        first_frame = again.detect({"rgb": rgb, "depth": depth, "K": k, "seconds": .1}, frame_id=7)
        next_frame = again.detect({"rgb": rgb, "depth": depth, "K": k, "seconds": .2}, frame_id=8)
        self.assertEqual({d.label: d.track_id for d in first_frame},
                         {d.label: d.track_id for d in next_frame})
        self.assertTrue(all(np.isfinite(d.position_base).all() for d in found))
        self.assertLess(found[0].position_base[1], found[1].position_base[1])

    def test_task_stack_keeps_empty_template_in_hold(self):
        stack = G1TaskStack({"wire_color_sequence": [], "medical_supply_order": [],
                             "perception": {}})
        frame = {"rgb": np.full((16, 16, 3), 80, dtype=np.uint8),
                 "depth": np.full((16, 16), 1.0, dtype=np.float32),
                 "K": np.array([[20., 0., 8.], [0., 20., 8.], [0., 0., 1.]]),
                 "seconds": 0.1}
        result = stack.update(frame, healthy())
        self.assertEqual(result["mission"]["action"], "hold")
        self.assertEqual(result["mission"]["state"], MissionState.WAIT_CONFIG.value)
        self.assertEqual(result["detections"], [])


if __name__ == "__main__":
    unittest.main()
