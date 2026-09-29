"""Measure motor power, test a relative turn, or regulate forward speed with the camera.

Examples (flash the accompanying firmware first; press g in the preview to begin):
  python motion_control.py 10.178.188.50 --camera 1 --test-duty .20 --duration 2
  python motion_control.py 10.178.188.50 --camera 1 --turn-deg 15
  python motion_control.py 10.178.188.50 --camera 1 --speed-mm-s 100 --duration 5

Positive turn = clockwise/right. q, x, SPACE or ESC stops and exits.
Defaults are starting limits, not a hardware calibration or a 1-degree guarantee.
See MOTION_CONTROL.md before tuning. Existing autonomy.py still uses legacy drive.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import threading
import time
import uuid

from motion_feedback import FeedbackRun, Measurement, Settings, VelocityEstimator, angle


class CameraFrames:
    """Drain camera buffers; keep one frame and its local receipt time.

    No background thread sends motor commands. If processing stops, firmware's
    300 ms watchdog expires. Local receipt time is not the camera exposure time.
    """
    def __init__(self, cap):
        self.cap, self.frame, self.t = cap, None, 0.0
        self.failed = False
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stop_event.is_set():
                ok, frame = self.cap.read()
                with self.lock:
                    if not ok:
                        self.failed = True
                        return
                    self.frame, self.t = frame, time.monotonic()
        except Exception:
            with self.lock:
                self.failed = True

    def read(self):
        with self.lock:
            return self.frame, self.t, self.failed

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=.5)


def ready_status(status, session, age):
    return (isinstance(status, dict) and age < .6 and status.get('session') == session
            and status.get('direct_pwm') == 1)


def clear_of_edges(m, cfg, margin):
    # A conservative circle around the axle covers the whole body during rotation.
    tag = cfg['robot_tag']
    fp = tag['footprint_mm']
    offset = float(tag.get('axle_offset_mm', 0))
    radius = max(math.hypot(offset + x, y) for x in (fp['front'], -fp['back'])
                 for y in (-fp['left'], fp['right']))
    w, h = cfg['arena']['size_mm']
    radius += margin
    return radius < m.x < w - radius and radius < m.y < h - radius


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('esp_ip', nargs='?')
    p.add_argument('--port', type=int, default=4211)
    p.add_argument('--camera', type=int, default=1)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    p.add_argument('--preview', action='store_true', help='camera/measurement only; opens no network connection')
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--turn-deg', type=float)
    mode.add_argument('--speed-mm-s', type=float)
    mode.add_argument('--test-duty', type=float, help='0..1 real PWM fraction, no minimum floor')
    p.add_argument('--test-motion', choices=['forward', 'left', 'right'], default='forward')
    p.add_argument('--duration', type=float, default=3)
    p.add_argument('--max-duty', type=float, default=.4)
    p.add_argument('--base-duty', type=float, default=0, help='optional measured running duty, not start/stall duty')
    p.add_argument('--tolerance-deg', type=float, default=1)
    p.add_argument('--camera-delay', type=float, default=.2, help='measured command-to-camera delay, seconds')
    p.add_argument('--turn-kp', type=float, default=.012)
    p.add_argument('--turn-kd', type=float, default=.001)
    p.add_argument('--speed-kp', type=float, default=.002)
    p.add_argument('--speed-ki', type=float, default=.002)
    p.add_argument('--max-turn-rate', type=float, default=90, help='abort above measured deg/s')
    p.add_argument('--max-speed', type=float, default=300, help='abort above measured mm/s')
    p.add_argument('--timeout', type=float, default=20)
    p.add_argument('--edge-margin', type=float, default=60, help='mm beyond full robot rotation footprint')
    p.add_argument('--log-dir', type=Path, default=Path('runs/motion'))
    return p


def run(args, settings, mode, value):
    import cv2
    from camera_io import apply_camera_properties
    from robot_pose import RobotPoseEstimator, draw
    from teleop import Link

    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    pose_est = RobotPoseEstimator(cfg)
    velocity = VelocityEstimator(float(cfg['robot_tag'].get('axle_offset_mm', 0)))
    if not cfg['robot_tag'].get('footprint_mm'):
        raise ValueError('robot_tag.footprint_mm is required for the edge check')
    cap = cv2.VideoCapture(args.camera)
    reader = link = log = None
    trial = last_measurement = None
    command = (0.0, 0.0)
    measured_speeds = []
    trial_id = time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6]
    run_dir = args.log_dir / trial_id
    run_dir.mkdir(parents=True)
    try:
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open camera {args.camera}')
        camera_readings = apply_camera_properties(cap, cfg)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        reader = CameraFrames(cap)
        if not args.preview:
            link = Link(args.esp_ip, args.port)
        (run_dir / 'meta.json').write_text(json.dumps(dict(
            args=vars(args), settings=asdict(settings), config=cfg, camera=camera_readings),
            indent=2, default=str), encoding='utf-8')
        log = (run_dir / 'trace.jsonl').open('w', encoding='utf-8')
        last_frame_t = last_send = last_print = 0.0
        arm_time = None
        label = 'PREVIEW ONLY' if args.preview else 'READY: press g to begin'
        print(label, '| Clear floor required; camera edge check is not obstacle avoidance.')
        while True:
            now = time.monotonic()
            if link:
                link.poll()
            frame, frame_t, failed = reader.read()
            fresh_frame = frame is not None and frame_t != last_frame_t
            m = None
            if failed:
                if trial:
                    trial.abort('camera_failed')
                break
            if fresh_frame:
                last_frame_t = frame_t
                pose = pose_est.detect(frame, t=frame_t)
                if pose is None:
                    velocity = VelocityEstimator(float(cfg['robot_tag'].get('axle_offset_mm', 0)))
                    last_measurement = None
                    if trial:
                        trial.abort('tag_lost')
                else:
                    m = velocity.update(pose)
                    last_measurement = m
                view = frame.copy()
                draw(view, pose_est, pose)
            else:
                pose = None
            now = time.monotonic()
            good_link = bool(link and ready_status(link.status, link.session, link.status_age()))
            good_pose = last_measurement is not None and now - last_measurement.t < .2
            if trial:
                if not good_pose:
                    trial.abort('pose_stale_or_missing')
                elif not good_link:
                    trial.abort('link_stale_or_firmware_missing')
                elif link.status.get('state') != 'RUNNING':
                    trial.abort('firmware_not_running')
                elif m is not None and not clear_of_edges(m, cfg, args.edge_margin):
                    trial.abort('too_close_to_arena_edge')
                elif m is not None and trial.phase not in ('done', 'aborted'):
                    command = trial.step(m)
                    if mode == 'speed' and trial.phase == 'running' and m.t - trial.start.t >= args.duration / 2:
                        measured_speeds.append(m.speed)
                if trial.phase in ('done', 'aborted'):
                    command = (0.0, 0.0)
                label = f'{trial.phase}: {trial.reason}'
            elif arm_time is not None:
                if now - arm_time > 2 or not good_pose or not good_link:
                    label, arm_time = 'Arming failed; check tag/link and press g again', None
                    link.send('stop')
                elif link.status.get('state') == 'RUNNING':
                    trial = FeedbackRun(mode, value, last_measurement, settings, args.duration, args.test_motion)
                    arm_time = None
                    label = 'running'
            if link and now - last_send >= .05:
                last_send = now
                if trial or arm_time is not None:
                    link.send('duty', l=round(command[0], 4), r=round(command[1], 4))
                else:
                    link.send('ping')
            if fresh_frame:
                row = dict(t=now, phase=trial.phase if trial else label, reason=trial.reason if trial else '',
                           measurement=asdict(m) if m else None, command=command,
                           controller=trial.debug if trial else {}, firmware=link.status if link else None)
                log.write(json.dumps(row) + '\n')
                lines = [label, f'{mode} target={value:g} | max PWM={settings.max_duty:.2f}',
                         f'v={m.speed:.1f} mm/s  w={m.omega:.1f} deg/s' if m else pose_est.last_reason,
                         'g begin | SPACE / q / x / ESC stop']
                if link and not good_link:
                    lines[0] = 'Waiting for matching firmware (direct_pwm=1) and session'
                for i, text in enumerate(lines):
                    cv2.putText(view, text, (10, 24 + 25 * i), 0, .55, (0, 0, 0), 3)
                    cv2.putText(view, text, (10, 24 + 25 * i), 0, .55, (255, 255, 255), 1)
                cv2.imshow('motion control', view)
            key = cv2.waitKey(1) & 255
            if key in (27, ord(' '), ord('q'), ord('x')):
                if trial:
                    trial.abort('operator_stop')
                break
            if key == ord('g') and not args.preview and trial is None and arm_time is None:
                if (good_pose and good_link and abs(last_measurement.speed) <= 5
                        and abs(last_measurement.omega) <= 1.5
                        and clear_of_edges(last_measurement, cfg, args.edge_margin)):
                    link.send('start')
                    arm_time = now
                else:
                    label = 'Cannot start: need still robot, fresh tag/link, and room to turn'
            if now - last_print > .5:
                print(label, trial.debug if trial else '', flush=True)
                log.flush()
                last_print = now
            if trial and trial.phase in ('done', 'aborted'):
                break
            time.sleep(.005)
    except KeyboardInterrupt:
        if trial:
            trial.abort('operator_interrupt')
        raise
    except Exception as exc:
        if trial:
            trial.abort('runtime_error: ' + str(exc))
        raise
    finally:
        # Stop commands are sent before joining the capture thread or closing files.
        if link:
            for _ in range(3):
                link.send('duty', l=0, r=0)
                link.send('stop')
                time.sleep(.02)
            link.sock.close()
        if reader:
            reader.stop()
        cap.release()
        if log:
            log.close()
        cv2.destroyAllWindows()
        summary = dict(result=trial.phase if trial else 'not_started', reason=trial.reason if trial else '',
                       target=value, mode=mode, preview=args.preview, hardware_accuracy_verified=False)
        if trial and last_measurement:
            summary.update(measured_turn_deg=angle(last_measurement.heading - trial.start.heading),
                           final_heading_error_deg=angle(trial.target_heading - last_measurement.heading))
        if measured_speeds:
            mean = sum(measured_speeds) / len(measured_speeds)
            summary.update(mean_speed_mm_s=mean, mean_speed_error_mm_s=mean - value,
                           mean_speed_within_10_percent=abs(mean - value) <= max(5, value * .1))
        (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print(json.dumps(summary, indent=2), '\nSaved:', run_dir.resolve())


def main():
    p = parser()
    args = p.parse_args()
    if not args.preview and not args.esp_ip:
        p.error('esp_ip is required, or use --preview (no network commands)')
    if not math.isfinite(args.edge_margin) or args.edge_margin < 0:
        p.error('edge-margin must be finite and nonnegative')
    mode, value = next((m, v) for m, v in (('turn', args.turn_deg), ('speed', args.speed_mm_s),
                                         ('duty', args.test_duty)) if v is not None)
    try:
        settings = Settings(max_duty=args.max_duty, base_duty=args.base_duty,
                            tolerance_deg=args.tolerance_deg, camera_delay_s=args.camera_delay,
                            turn_kp=args.turn_kp, turn_kd=args.turn_kd,
                            speed_kp=args.speed_kp, speed_ki=args.speed_ki,
                            max_omega_deg_s=args.max_turn_rate, max_speed_mm_s=args.max_speed,
                            timeout_s=args.timeout)
        FeedbackRun(mode, value, Measurement(0, 0, 0, 0, 0, 0), settings, args.duration, args.test_motion)
    except ValueError as exc:
        p.error(str(exc))
    run(args, settings, mode, value)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
