"""Drawing for the gesture apps: the driver dashboard and shared helpers (OpenCV only)."""
from functools import lru_cache

import cv2
import numpy as np

from gesture_logic import GRIP, MOTION, duty, FIRMWARE_MIN_DUTY

W, H = 1280, 720
FONT = cv2.FONT_HERSHEY_SIMPLEX
BG, PANEL, EDGE = (24, 20, 18), (40, 34, 30), (70, 62, 56)
WHITE, MUTED, DIM = (240, 238, 234), (160, 150, 140), (95, 88, 82)
GREEN, RED, AMBER, CYAN = (110, 210, 90), (80, 80, 235), (40, 175, 250), (220, 200, 60)
BRAND, ELEPHANT = 'ERA-ONE', '\U0001F418'
# Colour emoji fonts first (Windows, macOS, Linux), then a plain one that is tinted.
EMOJI_FONTS = ('C:/Windows/Fonts/seguiemj.ttf', '/System/Library/Fonts/Apple Color Emoji.ttc',
               '/usr/share/fonts/google-noto-color-emoji-fonts/NotoColorEmoji.ttf',
               '/usr/share/fonts/truetype/noto/NotoColorEmoji.ttf',
               '/usr/share/fonts/google-noto-emoji-fonts/NotoEmoji-Regular.ttf',
               '/usr/share/fonts/truetype/noto/NotoEmoji-Regular.ttf')
BONES = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
         (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17))


def kind_color(command):
    if command in MOTION:
        return GREEN
    if command in GRIP:
        return AMBER
    if command in ('STOP', 'CONFLICT'):
        return RED
    return MUTED


@lru_cache(maxsize=8)
def emoji(char, height, tint=CYAN):
    """BGRA picture of one emoji, `height` px tall, or None when no emoji font is installed."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return None
    for path in EMOJI_FONTS:
        for size in (109, 160, 4*height):       # colour bitmap fonts only exist at fixed sizes
            try:
                font = ImageFont.truetype(path, size)
            except OSError:
                continue
            canvas = Image.new('RGBA', (2*size, 2*size), (0, 0, 0, 0))
            ImageDraw.Draw(canvas).text((size//4, size//4), char, font=font, embedded_color=True,
                                        fill=(tint[2], tint[1], tint[0], 255))
            box = canvas.getbbox()
            if box is None:
                continue                         # font present but cannot draw it (e.g. COLRv1)
            glyph = canvas.crop(box)
            glyph = glyph.resize((max(1, round(glyph.width * height / glyph.height)), height), Image.LANCZOS)
            rgba = np.array(glyph)
            return np.dstack([rgba[..., 2::-1], rgba[..., 3]])
    return None


def paste(img, bgra, x, y):
    h, w = bgra.shape[:2]
    h, w = min(h, img.shape[0] - y), min(w, img.shape[1] - x)
    if h <= 0 or w <= 0:
        return
    alpha = bgra[:h, :w, 3:4] / 255.
    region = img[y:y+h, x:x+w]
    region[:] = (bgra[:h, :w, :3] * alpha + region * (1 - alpha)).astype(np.uint8)


def brand(img, subtitle):
    """'ERA-ONE <elephant> SUBTITLE' at the top left; returns the x where it ends."""
    put(img, BRAND, (16, 31), .85, CYAN, 2)
    x = 16 + cv2.getTextSize(BRAND, FONT, .85, 2)[0][0] + 10
    icon = emoji(ELEPHANT, 30)
    if icon is not None:
        paste(img, icon, x, 8)
        x += icon.shape[1] + 10
    put(img, subtitle, (x, 31), .6, WHITE)
    return x + cv2.getTextSize(subtitle, FONT, .6, 1)[0][0]


def put(img, text, org, scale=.5, color=WHITE, thick=1):
    cv2.putText(img, text, (int(org[0]), int(org[1])), FONT, scale, color, thick, cv2.LINE_AA)


def put_center(img, text, cx, y, scale=.5, color=WHITE, thick=1):
    (tw, _), _ = cv2.getTextSize(text, FONT, scale, thick)
    put(img, text, (cx - tw/2, y), scale, color, thick)


def card(img, x, y, w, h, title=None, color=PANEL):
    cv2.rectangle(img, (x, y), (x+w, y+h), color, -1)
    cv2.rectangle(img, (x, y), (x+w, y+h), EDGE, 1)
    if title:
        put(img, title, (x+12, y+20), .42, MUTED)


def bar(img, x, y, w, h, fraction, color, back=DIM):
    cv2.rectangle(img, (x, y), (x+w, y+h), back, -1)
    fill = int(round(w * max(0., min(1., fraction))))
    if fill:
        cv2.rectangle(img, (x, y), (x+fill, y+h), color, -1)


def place_camera(canvas, frame, x, y, w, h, banner=''):
    """Camera frame scaled into (x, y, w, h); a placeholder with banner text when missing."""
    if frame is not None:
        canvas[y:y+h, x:x+w] = cv2.resize(frame, (w, h))
    else:
        canvas[y:y+h, x:x+w] = (34, 29, 26)
        put_center(canvas, banner or 'No camera image', x + w/2, y + h/2, .7, MUTED)


def draw_hand(canvas, hand, x, y, w, h, color, label='', thick=2):
    """Skeleton of one hand (landmarks 0..1 in the mirrored frame) + a label tag at the wrist."""
    if len(hand.points) != 21:
        return
    pts = [(int(x + px*w), int(y + py*h)) for px, py in hand.points]
    for a, b in BONES:
        cv2.line(canvas, pts[a], pts[b], color, thick, cv2.LINE_AA)
    for p in pts:
        cv2.circle(canvas, p, 3 + (thick > 2), WHITE, -1, cv2.LINE_AA)
    if label:
        (tw, th), _ = cv2.getTextSize(label, FONT, .55, 1)
        wx, wy = pts[0]
        lx = int(min(max(wx - tw/2 - 6, x + 2), x + w - tw - 14))
        ly = int(min(wy + 14, y + h - th - 12))
        cv2.rectangle(canvas, (lx, ly), (lx + tw + 12, ly + th + 12), color, -1)
        put(canvas, label, (lx + 6, ly + th + 6), .55, (20, 20, 20))


# ---------------------------------------------------------------- driver dashboard

def _icon(img, command, cx, cy, color):
    if command in MOTION:
        dx, dy = {'FORWARD': (0, -1), 'BACK': (0, 1), 'LEFT': (-1, 0), 'RIGHT': (1, 0)}[command]
        cv2.arrowedLine(img, (int(cx - 55*dx), int(cy - 55*dy)), (int(cx + 55*dx), int(cy + 55*dy)),
                        color, 16, cv2.LINE_AA, tipLength=.5)
    elif command == 'STOP':
        a = np.arange(8) * np.pi/4 + np.pi/8
        poly = np.int32(np.c_[cx + 62*np.cos(a), cy + 62*np.sin(a)])
        cv2.fillPoly(img, [poly], color, cv2.LINE_AA)
        put_center(img, 'STOP', cx, cy + 10, .9, WHITE, 2)
    elif command in GRIP:
        spread = 34 if command == 'GRIP_OPEN' else 8
        for s in (-1, 1):
            tip = (int(cx + s*spread), int(cy - 50))
            base = (int(cx + s*20), int(cy + 30))
            cv2.line(img, base, tip, color, 12, cv2.LINE_AA)
        cv2.line(img, (int(cx - 30), int(cy + 34)), (int(cx + 30), int(cy + 34)), color, 12, cv2.LINE_AA)
    elif command == 'CONFLICT':
        cv2.circle(img, (int(cx), int(cy)), 58, color, 6, cv2.LINE_AA)
        put_center(img, '!', cx, cy + 22, 2, color, 6)
    else:
        cv2.circle(img, (int(cx), int(cy)), 58, color, 4, cv2.LINE_AA)
        put_center(img, '?' if command == 'NO_HAND' else '-', cx, cy + 18, 1.6, color, 4)


def render_drive(frame, hands, control, session, wheels, now, *, mode='PREVIEW', target='',
                 model='', fps=0., events=(), error=''):
    """Full dashboard (1280 x 720) for the real gesture controller."""
    img = np.full((H, W, 3), BG, np.uint8)
    command = control.command
    color = kind_color(command)
    moving = wheels != (0., 0.)

    # top bar
    bx = brand(img, 'GESTURE DRIVE') + 24
    badge = {'LIVE': RED, 'DEMO': AMBER}.get(mode, CYAN)
    btext = f'{mode} {target}'.strip()
    (bw, _), _ = cv2.getTextSize(btext, FONT, .5, 1)
    cv2.rectangle(img, (bx, 12), (bx + bw + 20, 38), badge, -1)
    put(img, btext, (bx + 10, 30), .5, (20, 20, 20))
    put(img, 'G start   SPACE/X stop   +/- speed   ESC quit', (W - 470, 31), .5, MUTED)

    # camera with hands
    cx0, cy0, cw, ch = 16, 52, 880, 660
    place_camera(img, frame, cx0, cy0, cw, ch, error)
    for i, hand in enumerate(hands):
        active = i in control.active
        tag = f'{hand.side or "?"}  {hand.pose.replace("_", " ")}' + (
            f'  {hand.score:.2f}' if hand.score is not None else '')
        draw_hand(img, hand, cx0, cy0, cw, ch, kind_color(hand.pose) if hand.pose != 'NONE' else MUTED,
                  tag, 4 if active else 2)
    frame_color = GREEN if moving else (DIM if not control.enabled else color if command != 'NONE' else EDGE)
    cv2.rectangle(img, (cx0 - 2, cy0 - 2), (cx0 + cw + 1, cy0 + ch + 1), frame_color, 3)
    strip = img[cy0 + ch - 44:cy0 + ch, cx0:cx0 + cw]
    strip[:] = (strip * .35).astype(np.uint8)
    put(img, control.message, (cx0 + 14, cy0 + ch - 15), .7, WHITE if control.enabled else MUTED, 2)
    ages = [now - h.captured_at for h in hands]
    put(img, f'{fps:4.1f} fps' + (f'   frame {min(ages)*1000:.0f} ms' if ages else ''),
        (cx0 + cw - 210, cy0 + 22), .48, WHITE)

    px, pw = 912, 352
    # command tile
    card(img, px, 52, pw, 236, 'COMMAND')
    _icon(img, command, px + pw/2, 150, color if control.enabled else DIM)
    put_center(img, command.replace('_', ' '), px + pw/2, 246, .8, color if control.enabled else MUTED, 2)
    bar(img, px + 16, 262, pw - 32, 10, control.progress, color)
    if not control.enabled:
        cv2.rectangle(img, (px + 1, 60), (px + pw - 1, 92), (60, 50, 44), -1)
        put_center(img, 'PAUSED - press G', px + pw/2, 83, .62, AMBER, 2)

    # speed
    floor = session.floor.get('m', FIRMWARE_MIN_DUTY)
    card(img, px, 298, pw, 100, 'SPEED   (- / + keys)')
    put(img, f'{control.speed:.1f}', (px + 16, 360), 1.3, WHITE, 3)
    for k in range(10):
        on = k < round(control.speed * 10)
        cv2.rectangle(img, (px + 100 + k*24, 336), (px + 100 + k*24 + 18, 364), CYAN if on else DIM, -1)
    put(img, f'~PWM {duty(control.speed, floor)*100:.0f}%  (floor {floor:.2f}'
        + ('' if session.floor else ', firmware') + ')', (px + 100, 386), .42, MUTED)

    # wheels
    card(img, px, 408, pw, 72, 'WHEELS')
    for row, (name, value) in enumerate((('L', wheels[0]), ('R', wheels[1]))):
        y = 436 + row*22
        put(img, name, (px + 16, y + 10), .48, WHITE)
        mid = px + 40 + (pw - 110)//2
        cv2.rectangle(img, (px + 40, y), (px + pw - 70, y + 12), DIM, -1)
        end = int(mid + value * (pw - 110)/2)
        cv2.rectangle(img, (min(mid, end), y), (max(mid, end), y + 12), GREEN if value else DIM, -1)
        cv2.line(img, (mid, y - 2), (mid, y + 14), WHITE, 1)
        put(img, f'{value:+.2f}', (px + pw - 62, y + 11), .45, WHITE)

    # hands
    card(img, px, 490, pw, 84, 'HANDS   (either hand drives)')
    if not hands:
        put(img, 'No hands in view', (px + 16, 540), .55, MUTED)
    role = 'DRIVING' if command in MOTION else 'GRIP' if command in GRIP else command
    for row, (i, hand) in enumerate(list(enumerate(hands))[:2]):
        y = 520 + row*28
        hc = kind_color(hand.pose) if hand.pose != 'NONE' else MUTED
        cv2.circle(img, (px + 26, y + 6), 11, hc, -1)
        put_center(img, hand.side or '?', px + 26, y + 11, .45, (20, 20, 20), 1)
        put(img, hand.pose.replace('_', ' '), (px + 46, y + 12), .55, hc, 1)
        bar(img, px + 180, y, 90, 12, hand.score or 0., hc)
        if i in control.active and control.enabled:
            put(img, role, (px + 280, y + 12), .42, color)

    # robot
    card(img, px, 584, pw, 62)
    status = session.link.status if session.link else None
    if session.link is None:
        state, sc, detail = 'not connected (preview)', MUTED, ''
    elif not status:
        state, sc, detail = 'no status', RED, ''
    else:
        age = now - session.link.status_at
        state = status.get('state', '?') + (' (STALE)' if age > .6 else '')
        sc = GREEN if state == 'RUNNING' else AMBER
        fw = status.get('min_duty')
        detail = (f'servo {(status.get("servo") or ["?"])[0]}'
                  + (f'   floor {fw:.2f}' if isinstance(fw, (int, float)) else '')
                  + f'   status {age*1000:.0f} ms')
    cv2.circle(img, (px + 22, 605), 8, sc, -1)
    put(img, 'ROBOT  ' + state, (px + 38, 611), .55, WHITE)
    put(img, detail, (px + 14, 636), .42, MUTED)

    # events / messages
    card(img, px, 656, pw, 56)
    put(img, (error or session.message)[:52], (px + 12, 676), .42, AMBER if error else WHITE)
    put(img, ('   '.join(events[-2:]) if events else f'model: {model}')[:56], (px + 12, 700), .4, MUTED)
    return img
