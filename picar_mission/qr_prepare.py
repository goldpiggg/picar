"""Rotate BASE toward the start QR's side and tilt the elbow, then wait without a transfer plan."""
from .arm_sequence import PoseMove, number


def side_view_targets(positions, limits, fallback):
    targets = dict(fallback)
    if positions is not None:
        for side in ('LEFT','RIGHT'):
            records = [positions['positions'][side+'-'+row]['approach'] for row in ('A','B')]
            if all(record is not None for record in records):
                target = sum(record['angles_deg']['base'] for record in records)/2.
                if number(target) and limits['base'][0] <= target <= limits['base'][1]:
                    targets[side] = target
    return targets


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
        self.scan_targets = []

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
        low, high = self.profile.limits['base']
        self.scan_targets = list(dict.fromkeys(max(low,min(high,self.target+offset)) for offset in (-10.,10.)))
        self.scan_targets = [value for value in self.scan_targets if value != self.target]

    def next_scan_target(self):
        return self.scan_targets[0] if self.scan_targets else None

    def scan(self, now):
        if self.state != 'READY' or not self.scan_targets:
            raise RuntimeError('QR_SCAN_NOT_READY')
        self.target = self.scan_targets.pop(0)
        speed = self.speed() if callable(self.speed) else self.speed
        # Change yaw only: do not apply the elbow offset again on each scan.
        self.move = PoseMove(self.driver, {'base':self.target}, speed, clock=lambda:now)
        self.started_at, self.state = now, 'MOVING'

    def tick(self, now):
        if self.state == 'MOVING' and now-self.started_at > self.move.motion.duration+15.:
            raise RuntimeError('QR_PREPARATION_TIMEOUT')
        if self.state == 'MOVING' and self.move.tick(now):
            self.state = 'READY'
        return self.state

    def interrupt(self):
        """The side QR codes were read mid-turn: stop here for the transfer."""
        self.driver.hold()
        if self.state == 'MOVING':
            self.state = 'INTERRUPTED'

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
