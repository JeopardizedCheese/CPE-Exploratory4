"""Exercise the runner without opening a camera, socket, window, or real robot."""
from contextlib import ExitStack, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import motion_control
from motion_feedback import Measurement, Settings


class RunnerTests(unittest.TestCase):
    def exercise(self, *, preview=False, old_firmware=False, lose_tag=False):
        clock, frames, sent = [0.0], [0], []
        frame = np.zeros((100, 100, 3), np.uint8)
        reader = MagicMock()
        def read():
            frames[0] += 1
            clock[0] += .05
            return frame, clock[0], False
        reader.read.side_effect = read
        estimator = MagicMock()
        estimator.last_reason = 'ok'
        def detect(frame, t):
            if lose_tag and frames[0] == 4:
                return None
            return SimpleNamespace(t=t, x=600, y=600, heading_deg=0)
        estimator.detect.side_effect = detect
        velocity = MagicMock()
        velocity.update.side_effect = lambda p: Measurement(p.t, p.x, p.y, p.heading_deg, 0, 0)
        link = MagicMock()
        link.session = '123456789012'
        link.status = {'state': 'IDLE', 'session': link.session}
        if not old_firmware:
            link.status['direct_pwm'] = 1
        link.status_age.return_value = .01
        def send(command, **fields):
            sent.append((command, fields))
            if command == 'start':
                link.status['state'] = 'RUNNING'
            if command == 'stop':
                link.status['state'] = 'IDLE'
        link.send.side_effect = send
        keys = iter((ord('g'), -1, -1, 27))
        with tempfile.TemporaryDirectory() as tmp, ExitStack() as stack:
            root = Path(tmp)
            cfg = {'arena': {'size_mm': [2100, 1200]}, 'robot_tag': {
                'footprint_mm': {'front': 210, 'back': 90, 'left': 75, 'right': 75}}}
            config = root / 'calib.json'
            config.write_text(json.dumps(cfg))
            args = motion_control.parser().parse_args(['127.0.0.1', '--turn-deg', '15',
                '--config', str(config), '--log-dir', str(root / 'runs')] + (['--preview'] if preview else []))
            stack.enter_context(patch('motion_control.time.monotonic', side_effect=lambda: clock[0]))
            stack.enter_context(patch('motion_control.time.sleep'))
            stack.enter_context(patch('motion_control.CameraFrames', return_value=reader))
            stack.enter_context(patch('motion_control.VelocityEstimator', return_value=velocity))
            stack.enter_context(patch('robot_pose.RobotPoseEstimator', return_value=estimator))
            stack.enter_context(patch('robot_pose.draw'))
            stack.enter_context(patch('cv2.VideoCapture'))
            stack.enter_context(patch('cv2.imshow'))
            stack.enter_context(patch('cv2.destroyAllWindows'))
            stack.enter_context(patch('cv2.waitKey', side_effect=lambda _: next(keys, 27)))
            link_factory = stack.enter_context(patch('teleop.Link', return_value=link))
            stack.enter_context(redirect_stdout(io.StringIO()))
            motion_control.run(args, Settings(), 'turn', 15)
            summary = json.loads(next((root / 'runs').glob('*/summary.json')).read_text())
            if preview:
                link_factory.assert_not_called()
        return sent, summary

    def test_preview_never_constructs_network_link(self):
        sent, summary = self.exercise(preview=True)
        self.assertEqual(sent, [])
        self.assertEqual(summary['result'], 'not_started')

    def test_old_firmware_cannot_start_or_receive_nonzero_duty(self):
        sent, _ = self.exercise(old_firmware=True)
        self.assertFalse(any(c == 'start' for c, _ in sent))
        self.assertFalse(any(c == 'duty' and (f['l'] or f['r']) for c, f in sent))

    def test_tag_loss_after_arming_cuts_power_and_sends_stop(self):
        sent, summary = self.exercise(lose_tag=True)
        self.assertTrue(any(c == 'duty' and f['l'] > 0 for c, f in sent))
        self.assertEqual(summary['result'], 'aborted')
        self.assertEqual(summary['reason'], 'tag_lost')
        self.assertEqual(sent[-1][0], 'stop')
        self.assertTrue(all(f['l'] == f['r'] == 0 for c, f in sent[-6:] if c == 'duty'))


if __name__ == '__main__':
    unittest.main()
