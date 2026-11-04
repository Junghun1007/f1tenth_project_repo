"""Pure freshness and mutually exclusive actuator output checks."""
import math


class Inputs:
    def __init__(self):
        self.values = {}

    def set(self, key, value, now):
        self.values[key] = (value, now)

    def get(self, key, default=math.nan):
        return self.values.get(key, (default, -math.inf))[0]

    def fresh(self, keys, now, timeout, after=-math.inf):
        return all(k in self.values and after <= self.values[k][1] <= now and
                   now-self.values[k][1] <= timeout and
                   math.isfinite(self.values[k][0]) for k in keys)


def motor_output(duty, brake, servo, mode, permitted, emergency, hold, stopped):
    if not permitted:
        return 'brake_current', hold if stopped else emergency
    if (not all(math.isfinite(v) for v in (duty, brake, servo)) or
            (duty > .001 and brake > .001) or
            not 0 <= servo <= 1 or not 0 <= duty <= .100001 or
            not 0 <= brake <= emergency+.000001 or mode not in ('duty', 'brake_current')):
        raise ValueError('Invalid or out-of-bounds actuator command')
    return mode, duty if mode == 'duty' else brake
