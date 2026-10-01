"""Pickup regressions, including poses/obstacles from the supplied camera recording."""
import json
import unittest
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np

from autonomy import Planner
from perception import Perception, robot_silhouette_mm
from pickup import jaw_error
from robot_pose import Pose, footprint_polygon_mm
from wall_guard import WallGuard
from tests.test_autonomy import config, pose
from tests.test_perception import config as vision_config, scene
from tests.test_vision import color


OPEN = {'state': 'RUNNING', 'servo': [0]}
CLOSE = ('grip', {'p': 'close'})


class CaptureTests(unittest.TestCase):
    def test_stone_at_jaw_base_stops_then_closes_in_all_pickup_states(self):
        stone = {'color': 2, 'x': 1090, 'y': 600}  # 30 mm behind nominal grip point
        for state in ('SEARCH', 'PARK', 'GOTO_STAGE', 'ALIGN', 'APPROACH', 'WALL_RECOVERY', 'WALL_BLOCKED'):
            with self.subTest(state=state):
                p = Planner(config())
                p.state = state
                self.assertEqual(p.step(0, pose(1000, 600, 0), [], [], OPEN,
                                        jaw_observations=[stone]), (0, 0, []))
                self.assertEqual(p.state, 'CAPTURE')
                self.assertEqual(p.step(.2, pose(1000, 600, 0, .2), [], [], OPEN,
                                        jaw_observations=[stone]), (0, 0, []))
                l, r, events = p.step(.4, pose(1000, 600, 0, .4), [], [], OPEN,
                                      jaw_observations=[stone])
                self.assertEqual((l, r, events), (0, 0, [CLOSE]))
                self.assertEqual((p.state, p.carrying), ('GRIP', 2))

    def test_actual_jaw_heading_overrides_old_approach_line(self):
        p = Planner(config())
        p.state, p.heading = 'APPROACH', 0.0
        stone = {'color': 2, 'x': 1000, 'y': 710}
        robot = pose(1000, 600, 90)
        self.assertAlmostEqual(jaw_error(robot, stone)[0], -10)
        self.assertEqual(p.step(0, robot, [], [stone], OPEN), (0, 0, []))
        self.assertEqual(p.state, 'CAPTURE')

    def test_predicted_pose_cannot_close_before_actual_jaws_reach_stone(self):
        p = Planner(config())
        p.state = 'APPROACH'
        stone = {'color': 2, 'x': 1170, 'y': 600, 'confidence': 1}
        p.lock.target, p.lock.last_seen = stone, 0
        p._predict = lambda q: pose(1050, 600, 0, q.t)
        self.assertEqual(p.step(0, pose(1000, 600, 0), [stone], [stone], OPEN), (0, 0, []))
        self.assertEqual(p.state, 'APPROACH')

    def test_coasting_out_of_capture_does_not_close_or_push_forward(self):
        p = Planner(config())
        stone = {'color': 2, 'x': 1120, 'y': 600, 'confidence': 1}
        p.step(0, pose(1000, 600, 0), [stone], [stone], OPEN)
        for t in (.2, .4, .6, .8, 1.0):
            l, r, events = p.step(t, pose(1070, 600, 0, t), [stone], [stone], OPEN)
            self.assertLessEqual(l, 0)
            self.assertLessEqual(r, 0)
            self.assertNotIn(CLOSE, events)
        self.assertEqual(p.state, 'BACKOFF')

    def test_invalid_feedback_never_closes_or_drives(self):
        for fields in ({'perception_status': 'reference_moved'}, {'perception_status': 'lighting_change'},
                       {'require_status': True}, {'status': {'state': 'IDLE'}}):
            p = Planner(config())
            stone = {'color': 2, 'x': 1120, 'y': 600}
            p.step(0, pose(1000, 600, 0), [], [stone])
            self.assertEqual(p.step(.5, pose(1000, 600, 0, .5), [], [],
                                    jaw_observations=[stone], **fields), (0, 0, []))
        self.assertEqual(p.step(.6, None, [], [], jaw_observations=[stone]), (0, 0, []))

    def test_wrong_close_angle_stays_stopped_even_at_wall(self):
        p = Planner(config())
        p.state, p.carrying = 'GRIP', 2
        wrong = {'state': 'RUNNING', 'servo': [40]}
        self.assertEqual(p.step(2.1, pose(100, 600, 180, 2.1), [], [], wrong), (0, 0, []))
        self.assertEqual(p.state, 'GRIP_BLOCKED')
        self.assertEqual(p.step(3, pose(100, 600, 180, 3), [], [], wrong), (0, 0, []))
        self.assertIn('angle mismatch', p.debug['reason'])

    def test_confirmed_70_degree_close_finishes_grip(self):
        p = Planner(config())
        p.state, p.carrying = 'GRIP', 2
        self.assertEqual(p.step(.4, pose(1000, 600, 0, .4), [], [],
                                {'state': 'RUNNING', 'servo': [70]}), (0, 0, []))
        self.assertEqual(p.state, 'CARRY')

    def test_stone_that_moves_out_of_jaws_is_remeasured_before_closing(self):
        p = Planner(config())
        stone = {'color': 2, 'x': 1120, 'y': 600}
        p.step(0, pose(1000, 600, 0), [], [stone], OPEN)
        moved = dict(stone, y=630)
        self.assertEqual(p.step(.4, pose(1000, 600, 0, .4), [], [moved], OPEN), (0, 0, []))
        self.assertEqual(p.state, 'APPROACH')
        self.assertEqual(p.lock.target, moved)

    def test_wall_pulse_is_cancelled_when_jaw_stone_preempts_recovery(self):
        p = Planner(config())
        p.step(0, pose(100, 600, 180), [], [], OPEN)
        self.assertLess(p.step(.4, pose(100, 600, 180, .4), [], [], OPEN)[0], 0)
        stone = {'color': 2, 'x': -20, 'y': 600}
        self.assertEqual(p.step(.42, pose(100, 600, 180, .42), [], [], OPEN,
                                jaw_observations=[stone]), (0, 0, []))
        self.assertIsNone(p.debug['wall_command_until'])
        self.assertEqual(p.state, 'CAPTURE')


class ApproachTests(unittest.TestCase):
    def approach(self, side=0, along=45):
        p = Planner(config())
        stone = {'color': 2, 'x': 1120+along, 'y': 600+side, 'confidence': 1}
        p.lock.target, p.lock.last_seen = stone, 0
        p._go('APPROACH', 0)
        p.step(0, pose(1000, 600, 0), [stone], [], OPEN)
        return p, stone

    def test_close_off_center_stone_causes_retreat_before_turning(self):
        p, stone = self.approach(side=35, along=10)
        l, r, events = p.step(.4, pose(1000, 600, 0, .4), [stone], [stone], OPEN)
        self.assertLess(l, 0)
        self.assertEqual(l, r)
        self.assertEqual(events, [])
        self.assertEqual(p.debug['reason'], 'pickup_retreat_before_turn')

    def test_obstacle_behind_blocks_realign_retreat(self):
        p, stone = self.approach(side=35, along=10)
        self.assertEqual(p.step(.4, pose(1000, 600, 0, .4), [stone],
                                [{'color': 0, 'x': 820, 'y': 600}], OPEN), (0, 0, []))
        self.assertEqual(p.debug['reason'], 'pickup_no_room_to_realign')

    def test_forward_pulse_has_deadline_then_stops_for_new_measurement(self):
        p, stone = self.approach()
        l, r, _ = p.step(.4, pose(1000, 600, 0, .4), [stone], [], OPEN)
        self.assertGreater(l, 0)
        self.assertEqual(l, r)
        deadline = p.debug['pickup_command_until']
        self.assertLessEqual(deadline-.4, .2)
        self.assertEqual(p.step(deadline+.01, pose(1010, 600, 0, deadline+.01), [stone], [], OPEN), (0, 0, []))
        self.assertNotIn('pickup_command_until', p.debug)

    def test_bad_or_missing_pose_cancels_pickup_pulse(self):
        for bad in (None, pose(float('nan'), 600, 0, .42), pose(1000, 600, 0, -1)):
            p, stone = self.approach()
            p.step(.4, pose(1000, 600, 0, .4), [stone], [], OPEN)
            self.assertGreater(p.pickup.until, .4)
            self.assertEqual(p.step(.42, bad, [stone], [], OPEN), (0, 0, []))
            self.assertEqual(p.pickup.until, 0)


class JawVisionTests(unittest.TestCase):
    def image(self):
        cfg = vision_config()
        cfg['robot_tag']['grip_offset_mm'] = [120, 0]
        cfg['vision']['edge_margin_mm'] = 100
        bg, frame = scene(with_robot=False)
        # Pose supplied independently so this tests the perception pipeline, not marker rendering.
        robot = pose(480, 300, 0)
        cv2.rectangle(frame, (294, 144), (303, 153), color(20), -1)
        p = Perception(cfg, bg)
        p.pose_est.detect = Mock(return_value=robot)
        return p, frame

    def test_jaw_color_survives_body_and_edge_masks_without_becoming_navigation_target(self):
        p, frame = self.image()
        for t in (0, .1, .2):
            snap = p.step(frame, t)
        self.assertEqual([o['color'] for o in snap.jaw_observations], [4])
        self.assertFalse([o for o in snap.observations+snap.targets if o['x'] > 500])

    def test_static_exclusion_still_applies_to_jaw_detection(self):
        p, frame = self.image()
        p.cfg['exclude_polygons'] = [[[280, 130], [315, 130], [315, 170], [280, 170]]]
        self.assertEqual(p._jaw_observations(frame, pose(480, 300, 0)), [])

    def test_ambiguous_color_does_not_close_gripper(self):
        p, frame = self.image()
        p.cfg['hsv']['2_test'] = p.cfg['hsv']['4_test']
        self.assertEqual(p._jaw_observations(frame, pose(480, 300, 0)), [])

    def test_invalid_vision_and_missing_tag_suppress_jaw_evidence(self):
        p, frame = self.image()
        p.detector.process = Mock(return_value=(frame, [], None, 'reference_moved'))
        self.assertEqual(p.step(frame, 0).jaw_observations, [])
        p.detector.process = Mock(return_value=(frame, [], None, 'ok'))
        p.pose_est.detect.return_value = None
        self.assertEqual(p.step(frame, .1).jaw_observations, [])


class WallPileTests(unittest.TestCase):
    def test_can_reverse_away_from_stone_already_in_padding(self):
        g = WallGuard(config())
        robot = pose(1000, 600, 0)
        stone = {'x': 1180, 'y': 600, 'color': 2}  # 10 mm ahead of physical footprint
        reverse = [g._translated(robot, -d) for d in range(0, 121, 10)]
        forward = [g._translated(robot, d) for d in range(0, 121, 10)]
        self.assertTrue(g._path_clear(reverse, [stone], None))
        self.assertFalse(g._path_clear(forward, [stone], None))
        self.assertFalse(g._path_clear(reverse, [{'x': 800, 'y': 600}], None))
        # Closing the diagonal gap to a corner is also an approach, even though
        # one coordinate's padding distance stays constant.
        self.assertFalse(g._path_clear(forward[:2], [{'x': 1185, 'y': 720}], None))

    def test_recorded_roof_and_gripper_blobs_no_longer_block_escape(self):
        fixture = json.loads((Path(__file__).parent/'fixtures/pickup_wall_recording.json').read_text())
        cfg = fixture['config']
        tag = cfg['robot_tag']
        # Keep the recorded physical footprint to isolate the image-mask correction.
        p = Perception(cfg, None)
        guard = WallGuard(cfg, cfg['autonomy'])
        for f in fixture['frames'][1:]:
            with self.subTest(frame=f['video_frame']):
                robot = Pose(**f['pose'])
                obs = f['observation_list']
                self.assertIsNone(guard._escape(robot, obs, 5))
                silhouette = np.float32(robot_silhouette_mm(robot, p.footprint, p.cam_xy, p.cam_h, tag['height_mm']))
                floor = np.float32(footprint_polygon_mm(robot, tag['footprint_mm']))
                retained, removed = [], []
                for ob in obs:
                    # Trace positions already had stone-height correction applied.
                    point = p.cam_xy+(np.array([ob['x'], ob['y']])-p.cam_xy)*p.cam_h/(p.cam_h-p.stone_h)
                    (removed if cv2.pointPolygonTest(silhouette, tuple(point), False) >= 0 else retained).append(ob)
                # Both roof blobs must disappear. The gripper blob in frame 1288
                # is 1 mm outside this deliberately old, undersized footprint;
                # the separation-aware path check permits moving away from it.
                self.assertGreaterEqual(len(removed), 2)
                self.assertTrue(any(cv2.pointPolygonTest(floor, (ob['x'], ob['y']), False) < 0 for ob in removed))
                command, reason, _ = guard._escape(robot, retained, 5)
                self.assertLess(command[0], 0)
                self.assertEqual(command[0], command[1])
                self.assertEqual(reason, 'reverse_inward')
                self.assertTrue(any(ob['color'] == 1 for ob in retained))


if __name__ == '__main__':
    unittest.main()
