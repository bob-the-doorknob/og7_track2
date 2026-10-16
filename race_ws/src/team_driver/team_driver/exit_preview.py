"""Use the existing path follower to certify aligned, easing turn exits."""

import math


class ExitPreview:
    def __init__(self, enabled=False, hold_s=.15, max_cross_track_m=.20,
                 max_heading_rad=.20, curvature_ratio=.60, max_speed_gain_mps=1.0,
                 horizon_m=6.0, max_curvature_inv_m=.30):
        values = (hold_s, max_cross_track_m, max_heading_rad, curvature_ratio,
                  max_speed_gain_mps, horizon_m, max_curvature_inv_m)
        if not all(math.isfinite(v) and v > 0 for v in values) or curvature_ratio >= 1:
            raise ValueError('exit preview limits must be finite, positive; ratio < 1')
        self.enabled = enabled
        self.hold_s = hold_s
        self.max_cross_track_m = max_cross_track_m
        self.max_heading_rad = max_heading_rad
        self.curvature_ratio = curvature_ratio
        self.max_speed_gain_mps = max_speed_gain_mps
        self.horizon_m = horizon_m
        self.max_curvature_inv_m = max_curvature_inv_m

    def trajectory(self, path, profile, guard, safety, position, yaw, speed,
                   steering, target_speed):
        if not self.enabled or guard is None:
            return None
        state = path.path_state(position, yaw)
        if (abs(state.cross_track_error) > self.max_cross_track_m
                or abs(state.heading_error) > self.max_heading_rad):
            return None
        curvature, targets = profile.future_features(position)
        steering_curvature = math.tan(steering * safety.max_steering_rad) / safety.wheelbase
        # The geometric follower does not model grip or yaw transients. Do not
        # lift braking limits in tight turns or during large steering corrections.
        if (abs(curvature[0]) > self.max_curvature_inv_m
                or abs(steering_curvature) > self.max_curvature_inv_m):
            return None
        easing = (abs(curvature[0]) >= .05
                  and abs(curvature[1]) <= self.curvature_ratio * abs(curvature[0]))
        aligned_straight = abs(curvature[0]) < .05 and abs(curvature[1]) <= .10
        # Keep momentum once straightened, while respecting upcoming braking.
        horizon = min(safety.horizon, self.horizon_m)
        preview_cap = math.sqrt(2 * safety.max_deceleration
                                * max(0, horizon - safety.stop_margin))
        if (not (easing or aligned_straight)
                or targets[1] < min(target_speed, preview_cap)):
            return None
        trajectory = path.preview_trajectory(
            position, yaw, max(speed, target_speed), steering,
            horizon=horizon, hold_s=self.hold_s,
            step=safety.path_step)
        if trajectory is None or not guard.certifies_trajectory(
                position, yaw, trajectory, footprint_m=safety.footprint_radius):
            return None
        return trajectory

    def target_speed(self, decision, safety):
        if not decision.preview_used:
            return decision.target_speed
        # Bound the increase over the old arc cap; never raise the new obstacle cap.
        old_limit = math.sqrt(2 * safety.max_deceleration
                              * max(0, decision.arc_clearance - safety.stop_margin))
        return min(decision.target_speed, old_limit + self.max_speed_gain_mps)
