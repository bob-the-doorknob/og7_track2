"""Behavioral checks for the LiDAR safety boundary."""

import math
import unittest

import numpy as np

from team_driver.safety import LidarSafety


class LidarSafetyTests(unittest.TestCase):
    def setUp(self):
        self.safety = LidarSafety()
        self.angles = np.linspace(-math.pi / 2, math.pi / 2, 361)
        self.clear = np.full(self.angles.size, np.inf)
        self.forward_index = self.angles.size // 2

    def test_no_return_with_valid_infinite_readings_keeps_baseline_speed(self):
        decision = self.safety.apply(self.clear, self.angles, 0.0, 1.8)
        self.assertTrue(decision.sensor_valid)
        self.assertEqual(decision.steering, 0.0)
        self.assertEqual(decision.target_speed, 1.8)

    def test_missing_forward_scan_stops_the_car(self):
        missing = np.full(self.angles.size, np.nan)
        decision = self.safety.apply(missing, self.angles, 0.0, 1.8)
        self.assertFalse(decision.sensor_valid)
        self.assertEqual(decision.target_speed, 0.0)

    def test_truncated_scan_stops_the_car(self):
        decision = self.safety.apply(
            np.array([2.0]), np.array([0.0]), 0.0, 1.8
        )
        self.assertFalse(decision.sensor_valid)
        self.assertEqual(decision.target_speed, 0.0)

    def test_close_forward_obstacle_stops_the_car(self):
        scan = self.clear.copy()
        scan[self.forward_index] = 0.20
        decision = self.safety.apply(scan, self.angles, 0.0, 1.8)
        self.assertTrue(decision.sensor_valid)
        self.assertEqual(decision.target_speed, 0.0)

    def test_candidate_steering_into_obstacle_reverts_to_baseline(self):
        scan = self.clear.copy()
        obstacle_angle = 0.55
        scan[np.argmin(np.abs(self.angles - obstacle_angle))] = 0.76
        decision = self.safety.apply(
            scan, self.angles, 0.0, 1.8,
            proposed_steering=0.8, proposed_target_speed=1.8,
        )
        self.assertTrue(decision.residual_rejected)
        self.assertEqual(decision.steering, 0.0)
        self.assertEqual(decision.target_speed, 1.8)

    def test_speed_proposal_cannot_exceed_baseline(self):
        decision = self.safety.apply(
            self.clear, self.angles, 0.0, 1.8,
            proposed_target_speed=3.0,
        )
        self.assertEqual(decision.target_speed, 1.8)

    def test_map_certified_clear_horizon_does_not_impose_scan_range_ceiling(self):
        guard = LidarSafety(horizon=10.0, max_range=10.0,
                            max_deceleration=2.0)
        ordinary = guard.apply(self.clear, self.angles, 0.0, 9.0)
        certified = guard.apply(self.clear, self.angles, 0.0, 9.0,
                                clear_horizon_certified=True)
        self.assertLess(ordinary.target_speed, 9.0)
        self.assertEqual(certified.target_speed, 9.0)
        obstacle = self.clear.copy()
        obstacle[self.forward_index] = 4.0
        blocked = guard.apply(obstacle, self.angles, 0.0, 9.0,
                              clear_horizon_certified=True)
        self.assertLess(blocked.target_speed, 9.0)

    def test_explicit_ceiling_allows_bounded_residual_speed(self):
        decision = self.safety.apply(
            self.clear, self.angles, 0.0, 1.0,
            proposed_target_speed=1.5, speed_ceiling=1.4,
        )
        self.assertEqual(decision.target_speed, 1.4)

    def test_nonfinite_proposal_reverts_to_baseline(self):
        decision = self.safety.apply(
            self.clear, self.angles, 0.1, 1.8,
            proposed_steering=math.nan, proposed_target_speed=math.inf,
        )
        self.assertTrue(decision.residual_rejected)
        self.assertEqual(decision.steering, 0.1)
        self.assertEqual(decision.target_speed, 1.8)


if __name__ == '__main__':
    unittest.main()
