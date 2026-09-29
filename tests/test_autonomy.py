import math
import unittest
import autonomy
from autonomy import Planner
from robot_pose import Pose

ZONES = {'3_crimson': (575, 165), '2_cyan': (1425, 195), '6_lime': (195, 540),
         '1_violet': (190, 1000), '5_skyblue': (870, 1050), '4_orange': (1470, 1050)}


def config(axle=0.0):
    return {'arena': {'size_mm': [2100, 1200], 'mm_per_px': 3},
            'zones': {k: {'center_mm': list(c), 'radius_mm': 130} for k, c in ZONES.items()},
            'robot_tag': {'grip_offset_mm': [120, 0], 'axle_offset_mm': axle, 'grip_calibrated': True,
                          'footprint_mm': {'front': 170, 'back': 110, 'left': 105, 'right': 105}},
            'vision': {'pile_mode': True, 'clearance_mm': 60, 'gripper_width_mm': 60, 'approach_length_mm': 80}}


NOISE = {'pose_noise_mm': 4, 'heading_noise_deg': 1.5, 'tag_dropout': 0.1, 'latency_s': 0.12, 'grip_success': 0.85}


def pose(x, y, heading, t=0.0, grip=120):
    h = math.radians(heading)
    return Pose(x, y, heading, x + grip * math.cos(h), y + grip * math.sin(h), 100, t)


class SimulatedRuns(unittest.TestCase):
    def test_scattered_clean(self):
        cfg = config()
        r = autonomy.run_sim(cfg, autonomy.scenario(cfg, 'scattered', 0), seconds=90)
        self.assertGreaterEqual(r['correct'], 3)
        self.assertEqual(r['wrong'], 0)

    def test_noisy_never_wrong(self):
        cfg = config()
        for seed in (0, 1):
            r = autonomy.run_sim(cfg, autonomy.scenario(cfg, 'scattered', seed), seconds=90, params=NOISE, seed=seed)
            self.assertEqual(r['wrong'], 0)
            self.assertGreaterEqual(r['correct'], 2)

    def test_axle_offset_known(self):
        cfg = config(axle=40)
        r = autonomy.run_sim(cfg, autonomy.scenario(cfg, 'scattered', 1), seconds=90, params={'axle_offset_mm': 40})
        self.assertGreaterEqual(r['correct'], 3)
        self.assertEqual(r['wrong'], 0)


class Safety(unittest.TestCase):
    def carrying_planner(self):
        p = Planner(config())
        p.state, p.since, p.carrying = 'CARRY', 0.0, 2           # carrying "cyan"
        p.pick_pos, p.pick_checked = (1000.0, 600.0), False
        return p

    def test_missed_grab_inside_other_zone_does_not_open_there(self):
        p = self.carrying_planner()
        still_there = [{'color': 2, 'x': 1000.0, 'y': 600.0}]
        inside_orange = pose(1470 - 120, 1050, 0)                # grip point at orange zone centre
        opened_in_zone = False
        for i in range(20):
            t = i * 0.1
            _, _, ev = p.step(t, pose(inside_orange.x, inside_orange.y, 0, t), [], still_there)
            opened_in_zone |= ('grip', {'p': 'open'}) in ev
        self.assertEqual(p.state, 'DISCARD')
        self.assertFalse(opened_in_zone)
        _, _, ev = p.step(3.0, pose(1000, 600, 0, 3.0), [], still_there)   # well away from zones
        self.assertIn(('grip', {'p': 'open'}), ev)

    def test_release_only_inside_own_zone(self):
        p = Planner(config())
        p.o['predict_pose'] = False             # the second pose is a jump, not a motion to extrapolate
        p.state, p.since, p.carrying, p.pick_checked = 'CARRY', 0.0, 4, True
        _, _, ev = p.step(5.0, pose(1000, 800, 0, 5.0), [], [], {'state': 'RUNNING', 'servo': [120]})
        self.assertNotIn(('grip', {'p': 'open'}), ev)
        self.assertEqual(p.state, 'CARRY')
        _, _, ev = p.step(5.1, pose(1470 - 120, 1050, 0, 5.1), [], [], {'state': 'RUNNING', 'servo': [120]})
        self.assertIn(('grip', {'p': 'open'}), ev)
        self.assertEqual(p.state, 'RELEASE')

    def test_never_sends_lift(self):
        cfg = config()
        r = autonomy.run_sim(cfg, autonomy.scenario(cfg, 'scattered', 0), seconds=30)
        self.assertGreaterEqual(r['correct'], 1)

    def test_stands_still_without_pose(self):
        p = Planner(config())
        self.assertEqual(p.step(1.0, None, [{'color': 1, 'x': 900, 'y': 600, 'confidence': .8}], [])[:2], (0.0, 0.0))
        self.assertEqual(p.step(1.0, pose(500, 500, 0, t=0.0), [], [])[:2], (0.0, 0.0))   # pose 1 s old

    def test_warning_until_calibrated(self):
        cfg = config()
        self.assertIsNone(autonomy.grip_calibration_warning(cfg))
        del cfg['robot_tag']['grip_calibrated']
        self.assertIn('calibrate_grip.py', autonomy.grip_calibration_warning(cfg))


class JawStone(unittest.TestCase):
    def heading_to_stage(self):
        p = Planner(config())
        far = {'color': 1, 'x': 400.0, 'y': 600.0, 'confidence': 1}
        p.lock.target, p.lock.last_seen = dict(far), 0.0
        p.state, p.since, p.heading = 'GOTO_STAGE', 0.0, math.pi   # stage point is behind the robot
        return p, far

    def test_stone_in_jaws_is_gripped_instead_of_driving_to_the_stage(self):
        p, far = self.heading_to_stage()
        robot = pose(1000, 600, 0)                                 # grip point at (1120, 600)
        in_jaws = {'color': 2, 'x': 1123.0, 'y': 604.0}           # not a target: never isolated
        _, _, events = p.step(0.1, robot, [far], [in_jaws, far])
        self.assertEqual(p.state, 'GRIP')
        self.assertIn(('grip', {'p': 'close'}), events)
        self.assertEqual(p.carrying, 2)
        self.assertEqual(p.pick_pos, (1123.0, 604.0))

    def test_unknown_colour_in_jaws_is_left_alone(self):
        p, far = self.heading_to_stage()
        p.step(0.1, pose(1000, 600, 0), [far], [{'color': 0, 'x': 1120.0, 'y': 600.0}, far])
        self.assertEqual(p.state, 'GOTO_STAGE')

    def test_stone_beside_the_jaws_is_not_a_jaw_stone(self):
        p, far = self.heading_to_stage()
        p.step(0.1, pose(1000, 600, 0), [far], [{'color': 2, 'x': 1120.0, 'y': 650.0}, far])
        self.assertEqual(p.state, 'GOTO_STAGE')


class CarryRoute(unittest.TestCase):
    def test_carry_goes_around_a_zone_on_the_straight_line(self):
        import sim
        cfg = config()
        robot = sim.SimRobot(cfg, [], 80, 800, 0)            # grip point (200, 800)
        robot.command('start', 0.0)
        p = Planner(cfg)
        p.o['camera_delay_s'] = 0.05                         # ideal sim: no camera delay to predict over
        p.state, p.since, p.carrying, p.pick_checked = 'CARRY', 0.0, 4, True   # to orange (1470, 1050)
        sky_x, sky_y = 870, 1050                              # straight line passes 118 mm from its centre
        closest, t = 1e9, 0.0
        while t < 40 and p.state == 'CARRY':
            robot.update(t)
            if round(t * 50) % 5 == 0:
                l, r, _ = p.step(t, robot.perceive(t)[0], [], [], robot.status(t))
            robot.command('drive', t, l=l, r=r)
            gx, gy = robot.grip_point()
            for x, y in ((gx, gy), (robot.x, robot.y)):
                closest = min(closest, math.hypot(x - sky_x, y - sky_y))
            t += 0.02
        self.assertEqual(p.state, 'RELEASE')
        self.assertGreater(closest, 130 + 60)                 # robot body stays out of the sky-blue zone

    def test_clear_line_goes_straight_to_the_zone(self):
        p = Planner(config())
        p.carrying = 4
        self.assertEqual(p._carry_waypoint(1470, 700, 1470, 1050), (1470, 1050))


class StallDiagnostic(unittest.TestCase):
    def feed(self, poses_and_cmds):
        p = Planner(config())
        return [p._stalled(t, pose(x, y, h, t), l, r) for t, x, y, h, l, r in poses_and_cmds]

    def test_spin_without_turning_is_flagged(self):
        out = self.feed([(i * .05, 800, 600, 10 + .1 * i, .3, -.3) for i in range(20)])
        self.assertIsNone(out[5])
        self.assertEqual(out[-1], 'spin')

    def test_drive_into_wall_is_flagged(self):
        out = self.feed([(i * .05, 800 + .3 * i, 600, 0, .3, .3) for i in range(20)])
        self.assertEqual(out[-1], 'drive')

    def test_moving_or_stopped_is_not_flagged(self):
        self.assertIsNone(self.feed([(i * .05, 800 + 10 * i, 600, 0, .3, .3) for i in range(20)])[-1])
        self.assertIsNone(self.feed([(i * .05, 800, 600, 3 * i, .3, -.3) for i in range(20)])[-1])
        self.assertIsNone(self.feed([(i * .05, 800, 600, 0, 0, 0) for i in range(20)])[-1])

    def test_diagnostic_does_not_change_commands(self):
        a, b = Planner(config()), Planner(config())
        b._stalled = lambda *args: 'spin'
        p = pose(400, 600, 0)
        stone = [{'color': 4, 'x': 900, 'y': 600, 'confidence': .9}]
        for t in (0.0, .05, .1, .15):
            p = pose(400, 600, 0, t)
            self.assertEqual(a.step(t, p, stone, stone)[:2], b.step(t, p, stone, stone)[:2])


class TurnPulses(unittest.TestCase):
    def test_pulse_settle_measure_cycle(self):
        p = Planner(config())
        err = math.radians(40)
        l, r = p._turn_step(err, pose(800, 600, 0, 0.0), 6)
        self.assertGreater(l, 0)
        self.assertEqual(l, -r)
        length = p._pulse['end'] - p._pulse['start']
        self.assertAlmostEqual(length, autonomy.pulse_seconds(40 * .7), places=3)   # 70% of 40 deg
        self.assertEqual(p._turn_step(err, pose(800, 600, 0, length / 2), 6), (l, r))        # still pulsing
        self.assertEqual(p._turn_step(err, pose(800, 600, 0, length + .1), 6), (0.0, 0.0))   # settling
        # after the settle the camera shows the whole pulse: 28 deg done, 12 to go -> next pulse
        l2, _ = p._turn_step(math.radians(12), pose(800, 600, 28, length + .3), 6)
        self.assertGreater(l2, 0)
        self.assertLess(p._pulse['end'] - p._pulse['start'], length)

    def test_done_only_at_rest(self):
        p = Planner(config())
        p._turn_step(math.radians(30), pose(800, 600, 0, 0.0), 6)
        self.assertIsNotNone(p._turn_step(math.radians(2), pose(800, 600, 28, 0.05), 6))   # mid-pulse
        self.assertIsNone(p._turn_step(math.radians(2), pose(800, 600, 28, 1.0), 6))       # measured, done

    def test_gain_learns_robot_turns_further(self):
        p = Planner(config())
        p._turn_step(math.radians(40), pose(800, 600, 0, 0.0), 6)
        planned = p._pulse['planned_deg']
        p._turn_step(math.radians(40), pose(800, 600, 2 * planned, 1.0), 6)   # turned twice as far
        self.assertGreater(p.pulse_gain, 1.2)

    def test_three_stuck_pulses_flag_a_stall(self):
        p = Planner(config())
        t = 0.0
        for _ in range(4):
            p._turn_step(math.radians(60), pose(800, 600, 0, t), 6)
            t += 1.0
        self.assertEqual(p._stuck_pulses, 3)
        self.assertEqual(p._stalled(t, pose(800, 600, 0, t), 0, 0), 'spin')
        self.assertLessEqual(p._pulse['end'] - p._pulse['start'], p.o['pulse_max_s'] + 1e-9)

    def test_pulse_cut_short_is_not_learned(self):
        p = Planner(config())
        p._turn_step(math.radians(60), pose(800, 600, 0, 0.0), 6)
        p.state = 'GRIP'                                  # a state that does not turn
        p.step(0.05, pose(800, 600, 0, 0.05), [], [])
        self.assertIsNone(p._pulse)
        self.assertEqual(p.pulse_gain, 1.0)

    def test_goal_behind_does_not_flip_turn_direction(self):
        p = Planner(config())
        p._was_spinning, p._spin_dir = True, 1.0
        l, r = p._drive_to(pose(800, 600, 0, .1), 800, 600, 400, 610, .3)   # err just past -180
        self.assertGreater(l, 0)

    def test_prediction_moves_pose_by_the_delay(self):
        p = Planner(config())
        for i in range(6):                                # 500 mm/s along +x, 30 fps
            t = i / 30
            p.motion.append((t, 800 + 500 * t, 600, 0, .3, .3))
        now = 6 / 30
        ahead = p._predict(pose(800 + 500 * now, 600, 0, now))
        self.assertAlmostEqual(ahead.x - (800 + 500 * now), 500 * p.o['camera_delay_s'], delta=2)
        still = Planner(config())
        still.motion = [(i / 30, 800, 600, 0, 0, 0) for i in range(6)]
        self.assertEqual(still._predict(pose(800, 600, 0, now)).x, 800)

    def test_backoff_stops_after_distance(self):
        p = Planner(config())
        p.o['predict_pose'] = False
        p._go('BACKOFF', 0.0)
        self.assertLess(p.step(0.0, pose(800, 600, 0, 0.0), [], [])[0], 0)
        self.assertLess(p.step(0.2, pose(750, 600, 0, 0.2), [], [])[0], 0)
        p.step(0.3, pose(800 - p.o['backoff_mm'] - 1, 600, 0, 0.3), [], [])
        self.assertEqual(p.state, 'SEARCH')

    def test_goal_inside_grip_circle_backs_out(self):
        p = Planner(config())
        pp = pose(800, 600, 0)
        l, r = p._drive_to(pp, pp.grip_x, pp.grip_y, 780, 600, .3)   # behind the grip point, near the axle
        self.assertLess(l, 0)
        self.assertEqual(l, r)

    def test_fixed_power_turns_still_available(self):
        cfg = config()
        cfg['autonomy'] = {'turn_mode': 'fixed'}
        self.assertEqual(Planner(cfg)._turn_step(math.radians(90), pose(800, 600, 0), 6)[0], .35)


class FieldPhysics(unittest.TestCase):
    def drive(self, l, r, secs, params, seed=0, **floor):
        import sim
        rb = sim.SimRobot(config(), [], 1000, 600, 0, params=dict(params, ramp_per_s=100), seed=seed)
        rb.command('start', 0)
        t = 0.0
        while t < secs:
            rb.command('drive', t, l=l, r=r, **floor)
            t += .02
            rb.update(t)
        return rb

    def test_default_physics_unchanged(self):
        rb = self.drive(.5, .5, 1.0, {})
        self.assertAlmostEqual(rb.x - 1000, 150, delta=1)

    def test_drive_packet_floor_replaces_min_duty(self):
        rb = self.drive(.5, .5, 1.0, {}, m=.5)                     # duty .5 + .5 * .5 = .75
        self.assertAlmostEqual(rb.x - 1000, 225, delta=2)
        self.assertEqual(rb.status(1.0)['min_duty'], .5)
        rb = self.drive(.5, .5, 1.0, {}, m=None)                   # no floor sent: MIN_DUTY
        self.assertAlmostEqual(rb.x - 1000, 150, delta=1)
        for bad in (1.5, -.1, 'x'):                                # like the firmware: refused
            rb = self.drive(.5, .5, 1.0, {}, m=bad)
            self.assertEqual((rb.x, rb.out), (1000, [0.0, 0.0]))

    def test_field_min_duty_speed(self):
        import sim
        rb = self.drive(.3, .3, 1.0, sim.FIELD_PARAMS)             # charged: ~530 mm/s
        self.assertTrue(480 < rb.x - 1000 < 580)
        rb = self.drive(.3, .3, 1.0, sim.LOW_BATTERY_PARAMS)       # flat battery: ~190 mm/s
        self.assertTrue(170 < rb.x - 1000 < 220)

    def test_field_spin_stalls_below_breakaway(self):
        import sim
        rb = self.drive(.1, -.1, 1.0, sim.LOW_BATTERY_PARAMS)
        self.assertEqual(rb.h, 0.0)
        rb = self.drive(.6, -.6, 1.0, sim.LOW_BATTERY_PARAMS)
        self.assertGreater(abs(rb.h), .5)

    def test_field_pulse_turns_like_the_robot(self):
        import sim
        turned = []
        for seed in range(30):
            rb = sim.SimRobot(config(), [], 1000, 600, 0, params=sim.FIELD_PARAMS, seed=seed)
            rb.command('start', 0)
            t = 0.0
            while t < .6:
                c = .5 if t < .15 else 0.0
                rb.command('drive', t, l=c, r=-c)
                t += .01
                rb.update(t)
            turned.append(math.degrees(rb.h))
        self.assertAlmostEqual(sorted(turned)[15], 20, delta=5)    # field: ~21 deg for 0.15 s

    def test_field_wall_and_tag_edge(self):
        import sim
        rb = self.drive(1, 1, 5.0, sim.FIELD_PARAMS)
        self.assertAlmostEqual(rb.x, 2100 - sim.FIELD_PARAMS['wall_mm'])
        self.assertIsNone(rb.perceive(5.0)[0])


class CircleFit(unittest.TestCase):
    def test_axle_circle(self):
        from calibrate_grip import fit_circle
        pts = [(500 + 40 * math.cos(a), 300 + 40 * math.sin(a)) for a in [i * 0.3 for i in range(21)]]
        cx, cy, r = fit_circle(pts)
        self.assertAlmostEqual(r, 40, delta=0.5)
        self.assertAlmostEqual(cx, 500, delta=0.5)


if __name__ == '__main__':
    unittest.main()