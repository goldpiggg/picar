"""Distance-to-PWM policy; never reverses or turns toward the QR direction."""


class Approach:
    def __init__(self, config):
        self.cfg = config
        self.slow_latched = False
        self.last_speed = None

    def command(self, reading):
        # Raw near readings take priority over the delayed median.
        distance = min(reading.raw_cm, reading.filtered_cm)
        if distance <= self.cfg.target_distance_cm:
            return 0
        if distance <= self.cfg.slow_distance_cm:
            self.slow_latched = True
        if not self.slow_latched:
            speed = self.cfg.cruise_speed
        else:
            ratio = max(0., min(1., (distance-self.cfg.target_distance_cm) /
                                (self.cfg.slow_distance_cm-self.cfg.target_distance_cm)))
            speed = round(self.cfg.slow_speed + ratio*(self.cfg.cruise_speed-self.cfg.slow_speed))
        # Once slowing starts, sensor noise cannot accelerate the car again.
        if self.last_speed is not None:
            speed = min(speed,self.last_speed)
        self.last_speed = speed
        return speed
