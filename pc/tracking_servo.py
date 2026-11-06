"""Short-lived, capture-aligned image goals for an open-loop position servo.

Angles here are calibrated PWM command estimates, never shaft feedback.
Each image replaces a finite goal. The faster command loop approaches it,
without repeatedly adding the same image error or building a long PWM queue.
"""
import math

AXES = ('pan', 'tilt')


class TrackingServo:
    def __init__(self):
        self.reset()

    def reset(self):
        self.goal = None
        self.received_at = None
        self.frame_time = None
        self.velocity = dict.fromkeys(AXES, 0.0)
        self.request = dict.fromkeys(AXES, None)
        self.active = dict.fromkeys(AXES, False)
        self.gains = dict.fromkeys(AXES, 0.0)
        self.status = 'idle'

    def observe(self, *, goal, capture_angles, rates, frame_time, now,
                max_rate_dps, max_accel_dps2):
        values = [frame_time, now, max_rate_dps, max_accel_dps2]
        values += [mapping[a] for mapping in (goal, capture_angles, rates) for a in AXES]
        if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values):
            raise ValueError('Tracking goal and timing must be finite')
        self.goal = dict(goal)
        self.received_at, self.frame_time = now, frame_time
        self.max_rate, self.max_accel = max_rate_dps, max_accel_dps2
        # A delayed observation cannot safely retain the high small-error
        # gain implied by a rate floor. This is a conservative gain budget,
        # not an assertion that an unknown mechanical plant is identified.
        delay = max(0., now - frame_time)
        gain_cap = min(4., 1. / (2. * (delay + .10)))
        for axis in AXES:
            correction = goal[axis] - capture_angles[axis]
            rate = rates[axis]
            self.active[axis] = abs(rate) > 1e-9 and abs(correction) > 1e-9
            self.gains[axis] = min(gain_cap, abs(rate / correction)) if self.active[axis] else 0.
            if not self.active[axis]:
                self.velocity[axis] = 0.
                self.request[axis] = None
        self.status = 'live'

    def step(self, *, now, elapsed, angles, max_age_s):
        if self.goal is None:
            return {}
        # No blind extrapolation after a missing/ambiguous observation. The
        # caller resets immediately in that case; this bounds a stalled feed.
        if now - self.received_at > .20 or now - self.frame_time > max_age_s:
            self.reset()
            self.status = 'waiting_for_frame'
            return {}
        dt = min(.05, max(0., elapsed))
        result = {}
        for axis in AXES:
            current = angles[axis]
            if current is None or not self.active[axis] or not dt:
                self.velocity[axis] = 0.
                self.request[axis] = None
                result[axis] = None  # Explicitly cancel pending motion.
                continue
            error = self.goal[axis] - current
            desired = max(-self.max_rate, min(self.max_rate, self.gains[axis] * error))
            previous_rate = self.velocity[axis]
            change = self.max_accel * dt
            rate = previous_rate + max(-change, min(change, desired - previous_rate))
            # Do not keep moving in the previous direction while the current
            # command has already reached/passed a newly captured goal.
            if rate * error <= 0:
                rate = 0.
            previous = current if self.request[axis] is None else self.request[axis]
            command = previous + rate * dt
            # PWM slew can be slower than the visual loop. At most one tick
            # of lead is retained; fractional command angles survive rounding.
            lead = max(.15, abs(rate) * dt)
            command = max(current - lead, min(current + lead, command))
            if error > 0: command = min(command, self.goal[axis])
            else: command = max(command, self.goal[axis])
            self.velocity[axis], self.request[axis] = rate, command
            result[axis] = command
        return result

    def snapshot(self):
        return dict(status=self.status, goal=self.goal, velocity_dps=dict(self.velocity),
                    effective_gain=dict(self.gains), max_hold_s=.20,
                    angle_source='calibrated_command_estimate; no shaft feedback')
