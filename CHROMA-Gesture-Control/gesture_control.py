"""ERA-ONE gesture drive: either hand shows a command (trained model). Default: preview, no network."""
import argparse
from pathlib import Path
import time

from gesture_logic import SPEED_MIN, GestureDrive, Hand, RobotLink
from gesture_link import GestureSession
from gesture_model import LABELS, GestureModel, newest_model
from hand_camera import CameraWorker

ROOT = Path(__file__).resolve().parent
WINDOW = 'ERA-ONE gesture drive'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0, help='Operator webcam index (not the field camera)')
    parser.add_argument('--live', action='store_true', help='Send UDP to the robot; otherwise preview only')
    parser.add_argument('--robot', help='Robot IP/hostname, required with --live')
    parser.add_argument('--port', type=int, default=4211)
    parser.add_argument('--speed', type=float, default=.3, help=f'Start speed {SPEED_MIN}..1 (+/- keys change it)')
    parser.add_argument('--min-duty', type=float,
                        help='Drive floor sent with every drive packet (e.g. .65); default = firmware MIN_DUTY')
    parser.add_argument('--model', type=Path, help='Trained model .npz (default: newest models/gesture_commands_*.npz)')
    parser.add_argument('--threshold', type=float, help="Override the model's confidence threshold (0.5..1)")
    parser.add_argument('--margin', type=float, help="Override the model's top-1 vs top-2 margin (0..1)")
    parser.add_argument('--demo', action='store_true', help='Keyboard hands, no camera, no network')
    args = parser.parse_args(argv)
    if args.demo and args.live:
        parser.error('--demo cannot send live robot commands')
    if args.live and not args.robot:
        parser.error('--live requires --robot ROBOT_IP')
    if not 1 <= args.port <= 65535:
        parser.error('Invalid port')
    try:
        control = GestureDrive(args.speed)
        GestureSession(control, None, args.min_duty)      # validate before opening a socket
    except ValueError as exc:
        parser.error(str(exc))
    model_name, classifier = 'demo (keyboard)', None
    if not args.demo:
        path = args.model or newest_model(ROOT/'models')
        if path is None or not path.is_file():
            parser.error('No trained model. Record with gesture_collect.py, then run gesture_train.py.')
        try:
            classifier = GestureModel(path, args.threshold, args.margin)
        except (OSError, ValueError, KeyError) as exc:
            parser.error(f'Cannot load model {path}: {exc}')
        model_name = f'{path.name}  (threshold {classifier.threshold:.2f}, margin {classifier.margin:.2f})'
    import cv2

    link = RobotLink(args.robot, args.port) if args.live else None
    session = GestureSession(control, link, args.min_duty)
    worker = None if args.demo else CameraWorker(args.camera, ROOT/'models/hand_landmarker.task', classifier)
    mode = 'LIVE' if args.live else 'DEMO' if args.demo else 'PREVIEW'
    demo = {'pose': None, 'second': False}
    events = []
    print(f'{mode}: model {model_name}')
    print('LIVE robot connection' if args.live else 'No UDP socket is opened')
    print('G start | SPACE/X stop | +/- speed | ESC quit'
          + (' | demo: 1-8 pose, 0 no hand, 9 second hand' if args.demo else ''))
    from gesture_ui import render_drive
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        if worker:
            worker.start()
        while True:
            now = time.monotonic()
            if args.demo:
                frame, error, fps = None, 'DEMO: keys 1-8 = pose, 0 = no hand, 9 = second hand (NONE)', 0.
                hands = () if demo['pose'] is None else (Hand(now, demo['pose'], .7, .5, score=1., side='R'),)
                if demo['second']:
                    hands += (Hand(now, 'NONE', .3, .5, score=1., side='L'),)
            else:
                frame, hands, error = worker.snapshot()
                fps = worker.fps
            left, right, fired = session.tick(hands)
            for _, argument in fired:
                events.append(f'{time.strftime("%H:%M:%S")} GRIP {argument.upper()}')
                print(events[-1])
            view = render_drive(frame, hands, control, session, (left, right), time.monotonic(), mode=mode,
                                target=args.robot or '', model=model_name, fps=fps, events=events, error=error)
            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(10) & 255
            if key == 27 or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord(' '), ord('x'), ord('X')):
                session.stop()
            elif key in (ord('g'), ord('G')):
                session.start()
            elif key in (ord('+'), ord('=')):
                print(f'speed {control.adjust_speed(+1):.1f}')
            elif key in (ord('-'), ord('_')):
                print(f'speed {control.adjust_speed(-1):.1f}')
            elif args.demo and ord('0') <= key <= ord('9'):
                if key == ord('9'):
                    demo['second'] = not demo['second']
                else:
                    demo['pose'] = None if key == ord('0') else LABELS[key - ord('1')]
    except KeyboardInterrupt:
        pass
    except (OSError, cv2.error) as exc:
        print(f'Control stopped: {exc}')
        return 1
    finally:
        if worker:
            worker.end.set()
        session.close()
        cv2.destroyAllWindows()
        if worker and worker.ident is not None:
            worker.join(timeout=1.)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
