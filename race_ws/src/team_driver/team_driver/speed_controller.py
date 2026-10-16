"""Bounded feed-forward plus PI speed control for normalized throttle."""

from __future__ import annotations

import math


def control_dt(mode: str, now: float, previous: float | None) -> float:
    """Choose a controller integration interval independent of ROS scheduling."""
    if mode == "nominal":
        return 0.025
    if mode != "wall":
        raise ValueError("control_dt_mode must be wall or nominal")
    return 0.025 if previous is None else max(0.0, now - previous)


class SpeedController:
    def __init__(
        self,
        feedforward_gain: float = 0.16,
        kp: float = 0.24,
        ki: float = 0.08,
        integral_limit: float = 2.0,
        throttle_min: float = -0.12,
        throttle_max: float = 0.65,
        active_braking: bool = False,
        approach_band_mps: float = 0.0,
    ) -> None:
        self.feedforward_gain = max(float(feedforward_gain), 0.0)
        self.kp = max(float(kp), 0.0)
        self.ki = max(float(ki), 0.0)
        self.integral_limit = max(float(integral_limit), 0.0)
        self.throttle_min = min(float(throttle_min), 0.0)
        self.throttle_max = max(float(throttle_max), 0.0)
        self.active_braking = bool(active_braking)
        self.approach_band_mps = float(approach_band_mps)
        if not math.isfinite(self.approach_band_mps) or self.approach_band_mps < 0.0:
            raise ValueError("approach_band_mps must be finite and nonnegative")
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def command(self, target_speed: float, speed: float, dt: float) -> float:
        """Convert a nonnegative speed target into normalized throttle.

        Reverse is limited to a small braking correction above the target and
        is never used to make up a speed deficit.
        """
        if not all(math.isfinite(value) for value in (target_speed, speed, dt)):
            self.reset()
            return 0.0
        target_speed = max(float(target_speed), 0.0)
        speed = max(float(speed), 0.0)
        dt = min(max(float(dt), 0.0), 0.2)
        error = target_speed - speed
        if self.active_braking and error < -0.05:
            self.integral = min(self.integral, 0.0)
            return max(self.throttle_min, -0.30 * (speed - target_speed))
        if dt > 0.0:
            self.integral = min(
                max(self.integral + error * dt, -self.integral_limit),
                self.integral_limit,
            )

        output = (
            self.feedforward_gain * target_speed
            + self.kp * error
            + self.ki * self.integral
        )
        # Prevent integrator windup while the actuator is saturated.
        if output > self.throttle_max and error > 0.0:
            self.integral = max(self.integral - error * dt, -self.integral_limit)
            output = self.throttle_max
        elif output < self.throttle_min and error < 0.0:
            self.integral = min(self.integral - error * dt, self.integral_limit)
            output = self.throttle_min

        # Do not drive backwards to slow the car; only use modest reverse torque
        # when the measured speed is materially above the requested speed.
        if speed <= target_speed + 0.20:
            output = max(output, 0.0)
        else:
            output = min(max(output, self.throttle_min), 0.0)
        if self.active_braking and self.approach_band_mps > 0.0 and output > 0.0:
            # Taper positive torque before reaching the target. Active braking
            # above the existing 0.05 m/s threshold returns earlier unchanged.
            # This avoids jumping from full feed-forward torque to light braking.
            output *= min(max(error / self.approach_band_mps, 0.0), 1.0)
        return min(max(float(output), self.throttle_min), self.throttle_max)
