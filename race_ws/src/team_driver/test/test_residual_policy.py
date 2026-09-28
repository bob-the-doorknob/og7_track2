"""Contract checks for the training/runtime residual interface."""

import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from team_driver.residual_policy import (
    MAX_SPEED_MULTIPLIER, MAX_STEERING_DELTA, MIN_SPEED_MULTIPLIER,
    OBSERVATION_DIM, ResidualPolicy, decode_action, pack_observation,
)


class ResidualPolicyTests(unittest.TestCase):
    def setUp(self):
        self.angles = np.linspace(-3 * math.pi / 4, 3 * math.pi / 4, 1080)
        self.ranges = np.full(self.angles.size, np.inf)

    def observation(self, **changes):
        values = dict(speed=2.0, heading_error=0.0, cross_track_error=0.0,
                      upcoming_curvature=0.0, baseline_steering=0.0,
                      baseline_target_speed=1.0)
        values.update(changes)
        return pack_observation(self.ranges, self.angles, **values)

    def test_observation_shape_and_normalization(self):
        obs = self.observation(heading_error=math.pi / 2,
                               cross_track_error=2.0)
        self.assertEqual(obs.shape, (OBSERVATION_DIM,))
        self.assertEqual(obs.dtype, np.float32)
        np.testing.assert_allclose(obs[:18], 1.0)
        np.testing.assert_allclose(obs[18:], (0.5, 0.5, 1.0, 0.0, 0.0, 0.25))

    def test_obstacle_sector_and_missing_sector(self):
        self.ranges[0] = 0.5
        self.assertAlmostEqual(self.observation()[0], 0.05)
        self.ranges[:60] = np.nan
        with self.assertRaisesRegex(ValueError, "LiDAR sector"):
            self.observation()

    def test_bad_state_rejected(self):
        with self.assertRaises(ValueError):
            self.observation(speed=math.nan)

    def test_action_bounds_and_zero_identity(self):
        self.assertEqual(decode_action((0.0, 0.0)), (0.0, 1.0))
        self.assertEqual(decode_action((4.0, 4.0)),
                         (MAX_STEERING_DELTA, MAX_SPEED_MULTIPLIER))
        self.assertEqual(decode_action((-4.0, -4.0)),
                         (-MAX_STEERING_DELTA, MIN_SPEED_MULTIPLIER))
        with self.assertRaises(ValueError):
            decode_action((math.nan, 0.0))

    @staticmethod
    def zero_weights():
        return dict(format_version=np.array(1), observation_version=np.array(1),
                    w0=np.zeros((64, OBSERVATION_DIM), np.float32),
                    b0=np.zeros(64, np.float32),
                    w1=np.zeros((64, 64), np.float32),
                    b1=np.zeros(64, np.float32),
                    w2=np.zeros((2, 64), np.float32),
                    b2=np.zeros(2, np.float32))

    def test_zero_weight_model_is_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "policy.npz"
            np.savez(path, **self.zero_weights())
            policy = ResidualPolicy.load(path)
            self.assertEqual(policy.predict(self.observation()), (0.0, 1.0))

    def test_bad_model_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "policy.npz"
            weights = self.zero_weights()
            weights["w0"][0, 0] = np.nan
            np.savez(path, **weights)
            with self.assertRaisesRegex(ValueError, "w0"):
                ResidualPolicy.load(path)
            weights = self.zero_weights()
            weights["observation_version"] = np.array(2)
            np.savez(path, **weights)
            with self.assertRaisesRegex(ValueError, "observation_version"):
                ResidualPolicy.load(path)


if __name__ == "__main__":
    unittest.main()
