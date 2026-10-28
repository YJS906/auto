"""ROS-independent command semantics and actuator envelope."""
from dataclasses import dataclass
import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class Envelope:
    mode: str = 'ackermann'
    wheelbase: float = 0.2
    central_steering_limit: float = 0.4
    max_speed: float = 0.6
    max_yaw_rate: float = 1.5
    acceleration: float = 0.5
    deceleration: float = 0.9
    steering_rate: float = 0.8
    speed: float = 0.0
    steering: float = 0.0

    def reset(self):
        self.speed = self.steering = 0.0

    def step(self, velocity, yaw_rate, dt):
        if not all(math.isfinite(x) for x in (velocity, yaw_rate, dt)) or dt <= 0:
            self.reset()
            return 0.0, 0.0
        target = clamp(velocity, -self.max_speed, self.max_speed)
        # A reversing command first brakes to zero; acceleration never bypasses deceleration.
        if self.speed * target < 0:
            target = 0.0
        limit = self.acceleration if abs(target) > abs(self.speed) else self.deceleration
        self.speed += clamp(target - self.speed, -limit*dt, limit*dt)
        if self.mode == 'diff':
            return self.speed, clamp(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)
        # Twist angular.z is yaw rate. At zero speed Ackermann cannot spin in place.
        desired = math.atan(self.wheelbase*yaw_rate/velocity) if abs(velocity) > 1e-4 else 0.0
        desired = clamp(desired, -self.central_steering_limit, self.central_steering_limit)
        self.steering += clamp(desired-self.steering, -self.steering_rate*dt, self.steering_rate*dt)
        yaw = self.speed*math.tan(self.steering)/self.wheelbase
        return self.speed, clamp(yaw, -self.max_yaw_rate, self.max_yaw_rate)
