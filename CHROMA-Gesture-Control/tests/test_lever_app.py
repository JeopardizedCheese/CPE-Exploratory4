import importlib.util
import unittest
from unittest.mock import patch

import lever_control


@unittest.skipUnless(importlib.util.find_spec('cv2'), 'Install requirements for UI smoke tests')
class AppTests(unittest.TestCase):
    def test_demo_runs_start_pause_quit_without_camera_or_socket(self):
        with patch('lever_control.CameraWorker', side_effect=AssertionError('demo opened camera')), \
             patch('lever_control.RobotLink', side_effect=AssertionError('preview opened socket')), \
             patch('cv2.namedWindow'), patch('cv2.setMouseCallback'), \
             patch('cv2.imshow') as show, patch('cv2.destroyAllWindows'), \
             patch('cv2.getWindowProperty', return_value=1), \
             patch('cv2.waitKey', side_effect=[ord('g'), ord('2'), ord('3'), ord('0'), ord('4'), ord('x'), 27]):
            self.assertEqual(lever_control.main(['--demo']), 0)
            self.assertEqual(show.call_count, 7)
            self.assertEqual(show.call_args.args[1].shape, (720, 1000, 3))

    def test_preview_default_has_no_socket(self):
        with patch('lever_control.CameraWorker') as worker_type, \
             patch('lever_control.RobotLink', side_effect=AssertionError('preview opened socket')), \
             patch('cv2.namedWindow'), patch('cv2.setMouseCallback'), \
             patch('cv2.imshow'), patch('cv2.destroyAllWindows'), \
             patch('cv2.waitKey', return_value=27):
            worker = worker_type.return_value
            worker.snapshot.return_value = (None, (), 'Simulated camera input')
            self.assertEqual(lever_control.main([]), 0)
            worker.start.assert_called_once()
            worker.end.set.assert_called_once()
            worker.join.assert_called_once()

    def test_demo_live_combination_is_rejected_before_socket_creation(self):
        with patch('lever_control.RobotLink', side_effect=AssertionError('socket opened')), \
             patch('sys.stderr'), self.assertRaises(SystemExit) as error:
            lever_control.main(['--demo', '--live', '--robot', '127.0.0.1'])
        self.assertEqual(error.exception.code, 2)

    def test_live_requires_explicit_robot(self):
        with patch('lever_control.RobotLink', side_effect=AssertionError('socket opened')), \
             patch('sys.stderr'), self.assertRaises(SystemExit) as error:
            lever_control.main(['--live'])
        self.assertEqual(error.exception.code, 2)

    def test_plus_minus_keys_change_speed_in_demo(self):
        with patch('lever_control.CameraWorker'), patch('lever_control.RobotLink'), \
             patch.object(lever_control.LeverControl, 'adjust_speed', autospec=True,
                          side_effect=lever_control.LeverControl.adjust_speed) as adjust, \
             patch('cv2.namedWindow'), patch('cv2.setMouseCallback'), patch('cv2.imshow'), \
             patch('cv2.destroyAllWindows'), patch('cv2.getWindowProperty', return_value=1), \
             patch('cv2.waitKey', side_effect=[ord('g'), ord('+'), ord('='), ord('-'), 27]):
            self.assertEqual(lever_control.main(['--demo']), 0)
        self.assertEqual([c.args[1] for c in adjust.call_args_list], [1, 1, -1])

    def test_min_duty_out_of_range_is_rejected(self):
        with patch('lever_control.RobotLink', side_effect=AssertionError('socket opened')), \
             patch('sys.stderr'), self.assertRaises(SystemExit) as error:
            lever_control.main(['--live', '--robot', '127.0.0.1', '--min-duty', '1.5'])
        self.assertEqual(error.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
