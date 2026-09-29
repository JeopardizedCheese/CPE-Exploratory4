"""Transport gate for the lever. Preview has no socket; live requires own status."""
import time


class LeverSession:
    def __init__(self, control, link=None):
        self.control, self.link = control, link
        self.pending_start = None
        self.last_start = self.last_send = float('-inf')
        self.last_wheels = (0., 0.)
        self.grip_retry = None
        self.message = 'Preview only: no robot connection' if link is None else 'LIVE: press G to request START'

    def start(self, now=None):
        now = time.monotonic() if now is None else now
        self.stop()
        if self.link is None:
            self.control.start()
            self.message = 'Preview enabled'
        else:
            self.link.poll()
            self.link.status = None  # never start on cached RUNNING status
            self.link.status_at = 0.
            self.link.send('start')
            self.pending_start = self.last_start = now
            self.message = 'Waiting for RUNNING status for this controller...'

    def stop(self):
        self.control.pause()
        self.pending_start = self.grip_retry = None
        self.last_wheels = (0., 0.)
        self.message = 'Stopped / paused; grip position is kept'
        if self.link:
            self.link.send('drive', l=0., r=0.)
            self.link.send('stop')

    def _running(self, now):
        return (self.link.running(now)
                and self.link.status.get('session') == self.link.session)

    def tick(self, hands, now=None):
        if self.link:
            self.link.poll()
        now = time.monotonic() if now is None else now
        if self.link:
            if self.pending_start is not None:
                if self._running(now) and self.link.status_at >= self.pending_start:
                    self.control.start()
                    self.pending_start = None
                    self.message = 'Robot RUNNING; center FIST to arm'
                elif now-self.pending_start > 1.5:
                    self.stop()
                    self.message = 'START not confirmed. Check IP/firmware; press G to retry'
                elif now-self.last_start >= .25:
                    self.link.send('start')
                    self.last_start = now
            if self.control.enabled and not self._running(now):
                self.stop()
                self.message = 'Robot status lost or session changed. Press G to restart'

        left, right, events = self.control.update(hands, now)
        if not self.control.enabled or self.control.mode != 'SERVO':
            self.grip_retry = None
        for command, argument in events:
            self.message = f'{command.upper()} {argument.upper()} requested (not physical confirmation)'
            if self.link:
                self.grip_retry = [argument, 3, now]

        if self.link:
            # A transition to zero bypasses the ordinary 20 Hz send schedule.
            stopped = (left, right) == (0., 0.) and self.last_wheels != (0., 0.)
            if stopped or now-self.last_send >= .05:
                self.link.send('drive', l=left, r=right)
                if not self.control.enabled and self.pending_start is None:
                    self.link.send('stop')
                self.last_send = now
            if self.grip_retry and now >= self.grip_retry[2]:
                # OPEN/CLOSE are idempotent targets. Three copies tolerate loss;
                # there is no physical grip sensor or per-command ACK here.
                self.link.send('drive', l=0., r=0.)
                self.link.send('grip', p=self.grip_retry[0])
                self.grip_retry[1] -= 1
                self.grip_retry[2] = now + .1
                if not self.grip_retry[1]:
                    self.grip_retry = None
        self.last_wheels = (left, right)
        return left, right, events

    def close(self):
        if self.link:
            try:
                for _ in range(3):
                    try:
                        self.stop()
                    except OSError:
                        pass
            finally:
                self.link.close()
        else:
            self.stop()
