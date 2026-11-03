"""Rotate BASE toward the start QR's side and tilt the elbow, then wait without a transfer plan."""
from .arm_sequence import PoseMove, number


class QrPreparation:
    def __init__(self, driver, profile, targets, speed, elbow_offset=0.):
        self.driver, self.profile, self.speed = driver, profile, speed
        self.base_targets = dict(targets)
        # Relative to the elbow at start, so the camera tilts the same amount
        # whatever reset pose the arm is in.
        self.elbow_offset = elbow_offset
        self.elbow_target = None
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
        if not self.elbow_offset:
            return {'base': target}
        elbow = self.driver.read_angles()['elbow']+self.elbow_offset
        low, high = self.profile.limits['elbow']
        if not number(elbow) or not low <= elbow <= high:
            raise ValueError('QR_ELBOW_TARGET_OUT_OF_LIMITS: %g' % elbow)
        return {'base': target, 'elbow': elbow}

    def start(self, direction, now):
        if self.state != 'IDLE':
            raise RuntimeError('QR preparation cannot restart')
        targets = self.targets(direction)
        self.driver.hold()
        self.direction, self.target = direction, targets['base']
        self.elbow_target = targets.get('elbow')
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
        return dict(state=self.state, direction=self.direction, base_target_deg=self.target,
                    elbow_target_deg=self.elbow_target)


class ArmReset(QrPreparation):
    """Restore the startup angles on arm channels only before scanning a new QR."""
    def __init__(self, driver, profile, targets, speed):
        super().__init__(driver, profile, {}, speed)
        self.reset_targets = dict(targets)

    def start(self, now):
        if self.state != 'IDLE':
            raise RuntimeError('Arm reset cannot restart')
        self.profile.check_angles(self.reset_targets)
        self.driver.hold()
        speed = self.speed() if callable(self.speed) else self.speed
        self.move = PoseMove(self.driver, self.reset_targets, speed, clock=lambda: now)
        self.started_at, self.state = now, 'MOVING'

    def snapshot(self):
        return dict(state=self.state, targets_deg=dict(self.reset_targets))
