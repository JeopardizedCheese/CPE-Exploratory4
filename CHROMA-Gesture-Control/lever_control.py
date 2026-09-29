"""CHROMA one-hand virtual lever v1. Default: webcam preview, no network."""
import argparse
from pathlib import Path
import time

from gesture_control import CameraWorker
from gesture_logic import Hand, RobotLink, duty
from lever_logic import LeverControl
from lever_link import LeverSession

ROOT = Path(__file__).resolve().parent
WINDOW = 'CHROMA | Virtual Lever v1'


def render_view(frame, hands, control, session, wheels, now, *, live=False,
                demo=False, error=''):
    import cv2
    import numpy as np
    view = np.full((720, 1000, 3), (29, 24, 20), np.uint8)
    if frame is not None:
        view[:480, :640] = cv2.resize(frame, (640, 480))
    else:
        view[:480, :640] = (43, 36, 30)
    cyan, white, muted = (220, 220, 70), (242, 240, 233), (173, 160, 144)
    orange = (80, 174, 255)

    def text(value, x, y, scale=.52, color=white):
        cv2.putText(view, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    servo = control.mode.startswith('SERVO')
    cx, cy = control.center
    radius = control.RETURN_ZONE if servo else control.DEAD_ZONE
    px, py = int(cx*640), int(cy*480)
    cv2.rectangle(view, (int((cx-radius)*640), int((cy-radius)*480)),
                  (int((cx+radius)*640), int((cy+radius)*480)), cyan, 2)
    cv2.drawMarker(view, (px, py), cyan, cv2.MARKER_CROSS, 16, 1)
    if servo:
        for dy, label in ((-control.SERVO_TRIGGER, 'CLOSE'), (control.SERVO_TRIGGER, 'OPEN')):
            yy = int((cy+dy)*480)
            cv2.line(view, (px-64, yy), (px+64, yy), orange, 2)
            text(label, min(px+70, 530), yy+5, color=orange)
    else:
        text('FORWARD', 265, 38, color=cyan)
        text('REVERSE', 266, 462, color=cyan)
        text('LEFT', 15, 244, color=cyan)
        text('RIGHT', 562, 244, color=cyan)
    for hand in hands:
        if 0 <= hand.x <= 1 and 0 <= hand.y <= 1:
            xx, yy = int(hand.x*640), int(hand.y*480)
            cv2.circle(view, (xx, yy), 13, white, 2)
            text(hand.pose, min(xx+18, 505), max(yy-18, 65), color=white)

    text('CHROMA', 668, 42, .9, cyan)
    text('VIRTUAL LEVER / V1', 668, 73, .5, muted)
    text('LIVE ROBOT' if live else 'DEMO / NO NETWORK' if demo else 'PREVIEW / NO NETWORK',
         668, 112, .51, orange if live else cyan)
    text(control.mode, 668, 165, .85, orange if servo else white)
    text('Enabled' if control.enabled else 'Paused - press G', 668, 197, .55, muted)
    text(f'L {wheels[0]:+.2f}     R {wheels[1]:+.2f}', 668, 240, .64)
    text(f'Command level: {control.speed:.2f}', 668, 275, .48)
    text(f'Legacy PWM at this level: {duty(control.speed)*100:.1f}%', 668, 301, .43, orange)
    text('Command level is NOT measured speed.', 668, 326, .40, muted)
    age = max((now-h.captured_at for h in hands), default=None)
    text(f'Frame age: {age*1000:.0f} ms' if age is not None else 'Hand: not detected', 668, 365, .5)
    if len(hands) == 1 and hands[0].score is not None:
        text(f'Pose score: {hands[0].score:.2f}', 668, 389, .5)
    status = session.link.status if session.link else None
    state = status.get('state', '?') if status else '-'
    servo_angle = (status.get('servo') or ['?'])[0] if status else '-'
    text(f'Robot: {state}', 668, 421, .48)
    text(f'Servo software angle: {servo_angle}', 668, 445, .44, muted)
    text('(not grip confirmation)', 668, 467, .4, muted)

    cv2.line(view, (20, 494), (980, 494), muted, 1)
    text(control.message, 24, 525, .57, cyan)
    text(error or session.message, 24, 555, .48, orange if error else muted)
    text('G start  |  SPACE / X stop  |  ESC quit  |  Open palm cancels motion', 24, 599, .53)
    text('FIST: move whole hand to drive. Center = stop. THUMB UP: hold to enter servo.', 24, 628, .49)
    text('SERVO: keep thumb up; move hand UP to CLOSE, DOWN to OPEN; center to rearm.', 24, 655, .49)
    if demo:
        text('DEMO: mouse in left panel = hand position | 1 fist | 2 thumb | 3 open | 0 no hand | 4 two hands',
             24, 691, .45, orange)
    else:
        text('Mirrored camera: UP/DOWN are image directions, not camera depth. Keep only one hand visible.',
             24, 691, .44, muted)
    return view


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0, help='Operator webcam index')
    parser.add_argument('--live', action='store_true', help='Send UDP to the robot; otherwise preview only')
    parser.add_argument('--robot', help='Explicit robot IP/hostname required with --live')
    parser.add_argument('--port', type=int, default=4211)
    parser.add_argument('--speed', type=float, default=.15,
                        help='Normalized command, NOT PWM or measured speed (default .15)')
    parser.add_argument('--min-score', type=float, default=.7)
    parser.add_argument('--demo', action='store_true', help='Mouse/key simulation without camera or network')
    args = parser.parse_args(argv)
    if args.demo and args.live:
        parser.error('--demo cannot send live robot commands')
    if args.live and not args.robot:
        parser.error('--live requires --robot YOUR_ROBOT_IP')
    if not 1 <= args.port <= 65535 or not 0 < args.min_score <= 1:
        parser.error('Invalid port or min-score')
    try:
        control = LeverControl(args.speed)
    except ValueError as exc:
        parser.error(str(exc))
    recognizer = ROOT/'models/gesture_recognizer.task'
    if not args.demo and not recognizer.is_file():
        parser.error('Missing gesture_recognizer.task. Run setup_lever.ps1 first.')
    try:
        import cv2
    except ImportError:
        parser.error('OpenCV is missing. Run setup_lever.ps1 first.')
    link = RobotLink(args.robot, args.port) if args.live else None
    session = LeverSession(control, link)
    worker = None if args.demo else CameraWorker(args.camera, ROOT/'models/hand_landmarker.task',
                                               recognizer=recognizer, min_score=args.min_score)
    demo_hand = {'x': .5, 'y': .5, 'pose': 'FIST', 'count': 1}

    def mouse(event, x, y, flags, param):
        if args.demo and 0 <= x < 640 and 0 <= y < 480:
            demo_hand.update(x=x/640, y=y/480)

    print('LIVE robot connection' if args.live else 'PREVIEW ONLY: no UDP socket is opened')
    print('Press G, then hold FIST in the center box. SPACE/X stop; ESC quits.')
    print('UP in image = forward/close; DOWN = reverse/open. No depth estimation.')
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, mouse)
        if worker:
            worker.start()
        while True:
            now = time.monotonic()
            if args.demo:
                frame, error = None, ''
                hands = tuple(Hand(now, demo_hand['pose'], demo_hand['x'], demo_hand['y'])
                              for _ in range(demo_hand['count']))
            else:
                frame, hands, error = worker.snapshot()
            left, right, events = session.tick(hands)
            for command, argument in events:
                print(f'{command.upper()} {argument.upper()} requested')
            view = render_view(frame, hands, control, session, (left, right), time.monotonic(),
                               live=args.live, demo=args.demo, error=error)
            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(10) & 255
            if key == 27 or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord(' '), ord('x'), ord('X')):
                session.stop()
            elif key in (ord('g'), ord('G')):
                session.start()
            elif args.demo and key in (ord('0'), ord('1'), ord('2'), ord('3'), ord('4')):
                if key in (ord('0'), ord('4')):
                    demo_hand['count'] = 0 if key == ord('0') else 2
                else:
                    demo_hand.update(count=1, pose={ord('1'): 'FIST', ord('2'): 'THUMB_UP', ord('3'): 'OPEN'}[key])
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
