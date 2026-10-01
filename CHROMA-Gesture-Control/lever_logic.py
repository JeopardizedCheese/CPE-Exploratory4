"""One-hand virtual lever. All decisions are independent of camera/network I/O."""
import math

from gesture_logic import _Track


class LeverControl:
    HOME = (.5, .5)
    DEAD_ZONE = .10
    RETURN_ZONE = .07
    SERVO_TRIGGER = .14
    ARM_SECONDS = .5
    CENTER_SECONDS = .3
    ACTION_SECONDS = .15
    MIN_SPEED, SPEED_STEP = .02, .05

    def __init__(self, speed=.15, timeout=.2):
        # Turning uses 0.6 * speed; keep it above firmware's 0.01 cutoff.
        if not math.isfinite(speed) or not self.MIN_SPEED <= speed <= 1:
            raise ValueError('speed must be in [0.02, 1]')
        if not 0 < timeout <= .3:
            raise ValueError('timeout must be in (0, 0.3]')
        self.speed = speed
        self.timeout = timeout
        self._track = _Track(timeout)
        self.enabled = False
        self.reset('Press G to enable preview/control')

    def adjust_speed(self, steps):
        """+/- keys: change the command level by SPEED_STEP, at once and in any mode."""
        self.speed = round(min(1., max(self.MIN_SPEED, self.speed + steps*self.SPEED_STEP)), 2)
        return self.speed

    def reset(self, message):
        self.mode = 'STOP'
        self.center = self.HOME
        self.message = message
        self.servo_ready = False
        self._anchor = self._since = None
        self._action = self._action_since = None
        self._track.reset()

    def start(self):
        self.enabled = True
        self.reset('Hold FIST in the center box for 0.5s')

    def pause(self):
        self.enabled = False
        self.reset('Paused. Press G to start again')

    def _stable(self, hand, seconds):
        """Movement restarts dwell; only capture time can advance it."""
        point = (hand.x, hand.y)
        if self._anchor is None or math.dist(point, self._anchor) > .035:
            self._anchor, self._since = point, hand.captured_at
        return hand.captured_at - self._since >= seconds

    def _clear_dwell(self):
        self._anchor = self._since = None
        self._action = self._action_since = None

    def update(self, hands, now):
        """Return normalized (left, right, [(command, argument), ...])."""
        zero = (0., 0., [])
        if not self.enabled:
            return zero
        if len(hands) != 1:
            self.reset('Show exactly ONE hand; then center FIST to rearm')
            return zero
        hand = hands[0]
        usable, new_frame, _ = self._track.observe(hand, now)
        if not usable:
            self.reset('Hand lost/stale; center FIST to rearm')
            return zero
        if hand.pose not in ('FIST', 'THUMB_UP'):
            self.reset('Stopped. Center FIST to rearm; grip position is kept')
            return zero

        if self.mode == 'STOP':
            centered = max(abs(hand.x-.5), abs(hand.y-.5)) <= self.DEAD_ZONE
            if hand.pose != 'FIST' or not centered:
                self._clear_dwell()
                self.message = 'Hold FIST in the center box for 0.5s'
            elif self._stable(hand, self.ARM_SECONDS) and new_frame:
                self.center = self._anchor
                self.mode = 'DRIVE'
                self._clear_dwell()
                self.message = 'DRIVE ready. Move FIST; center = stop'
            else:
                self.message = 'Keep FIST still: arming...'
            return zero

        if self.mode == 'DRIVE' and hand.pose == 'THUMB_UP':
            # Stop on the FIRST thumb frame, before the mode-entry dwell.
            self.mode = 'SERVO_ENTRY'
            self._clear_dwell()

        if self.mode == 'SERVO_ENTRY':
            if hand.pose != 'THUMB_UP':
                self.reset('Mode change cancelled. Center FIST to rearm')
            elif not (.2 <= hand.x <= .8 and .2 <= hand.y <= .8):
                self._clear_dwell()
                self.message = 'Keep THUMB UP; move away from image edges to set center'
            elif self._stable(hand, self.ARM_SECONDS) and new_frame:
                # A fresh reference prevents the entry motion from gripping.
                self.center = self._anchor
                self.mode = 'SERVO_CENTER'
                self._clear_dwell()
                self.message = 'SERVO: hold thumb in the new center box for 0.3s'
            else:
                self.message = 'Wheels stopped. Keep THUMB UP still for 0.5s'
            return zero

        dx, dy = hand.x-self.center[0], self.center[1]-hand.y
        if self.mode == 'DRIVE':
            if max(abs(dx), abs(dy)) <= self.DEAD_ZONE:
                self.message = 'DRIVE: neutral / wheels stopped'
                return zero
            if abs(dy) >= abs(dx):
                v = self.speed if dy > 0 else -self.speed
                self.message = 'DRIVE: FORWARD' if dy > 0 else 'DRIVE: REVERSE'
                return v, v, []
            v = self.speed * .6
            self.message = 'DRIVE: RIGHT' if dx > 0 else 'DRIVE: LEFT'
            return (v, -v, []) if dx > 0 else (-v, v, [])

        # All servo states keep both wheels at zero; folding the thumb cancels.
        if hand.pose != 'THUMB_UP':
            self.reset('Servo mode cancelled. Center FIST to rearm')
            return zero
        centered = max(abs(dx), abs(dy)) <= self.RETURN_ZONE
        if centered:
            self._action = self._action_since = None
            if self._stable(hand, self.CENTER_SECONDS) and new_frame:
                self.mode, self.servo_ready = 'SERVO', True
                self.message = 'SERVO ready: UP close / DOWN open (whole hand)'
            else:
                self.message = 'SERVO: settle in center to enable one command'
            return zero
        self._anchor = self._since = None
        if self.mode == 'SERVO_CENTER' or not self.servo_ready:
            self.message = 'SERVO: return to center before another command'
            return zero
        action = None
        if abs(dx) <= self.DEAD_ZONE and abs(dy) >= self.SERVO_TRIGGER:
            action = 'close' if dy > 0 else 'open'
        if action is None:
            self._action = self._action_since = None
            self.message = 'SERVO: move vertically; sideways movement is ignored'
            return zero
        if action != self._action:
            self._action, self._action_since = action, hand.captured_at
        if new_frame and hand.captured_at-self._action_since >= self.ACTION_SECONDS:
            self.servo_ready = False
            self._action = self._action_since = None
            self.message = f'GRIP {action.upper()} requested. Return to center'
            return 0., 0., [('grip', action)]
        self.message = f'SERVO: hold {action.upper()} briefly...'
        return zero
