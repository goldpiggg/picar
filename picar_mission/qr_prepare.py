"""Rotate BASE toward the start QR's side, then wait without a transfer plan."""
from .arm_sequence import PoseMove, number


class QrPreparation:
    def __init__(self, driver, profile, targets, speed):
        self.driver, self.profile, self.speed = driver, profile, speed
        self.base_targets = dict(targets)
        self.state = 'IDLE'
        self.move = None
        self.direction = None
        self.target = None
        self.started_at = None

    def targets(self, direction):
        if direction not in ('LEFT', 'RIGHT'):
            raise ValueError('Direction must be LEFT or RIGHT')
        target = self.base_targets[direction]
        low, high = self.profile.limits['base']
        if not number(target) or not low <= target <= high:
            raise ValueError('QR_BASE_TARGET_OUT_OF_LIMITS: '+direction)
        return {'base': target}

    def start(self, direction, now):
        if self.state != 'IDLE':
            raise RuntimeError('QR preparation cannot restart')
        targets = self.targets(direction)
        self.driver.hold()
        self.direction, self.target = direction, targets['base']
        speed = self.speed() if callable(self.speed) else self.speed
        self.move = PoseMove(self.driver, targets, speed, clock=lambda: now)
        self.started_at = now
        self.state = 'MOVING'

    def tick(self, now):
        if self.state == 'MOVING' and now-self.started_at > self.move.motion.duration+15.:
            raise RuntimeError('QR_PREPARATION_TIMEOUT')
        if self.state == 'MOVING' and self.move.tick(now):
            self.state = 'READY'
        return self.state

    def cancel(self):
        self.driver.hold()
        if self.state == 'MOVING':
            self.state = 'CANCELLED'

    def snapshot(self):
        return dict(state=self.state, direction=self.direction, base_target_deg=self.target)
