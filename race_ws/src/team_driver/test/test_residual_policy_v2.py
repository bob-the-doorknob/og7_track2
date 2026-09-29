"""The faster baseline uses versioned, high-speed observations."""

import math
import numpy as np
import pytest

from team_driver.residual_policy import (
    OBSERVATION_DIM_V2, ResidualPolicy, pack_observation_v2,
)


def test_v2_observation_has_future_curvature_and_speed_targets():
    angles = np.linspace(-3 * math.pi / 4, 3 * math.pi / 4, 1080)
    ranges = np.full(angles.shape, np.inf)
    obs = pack_observation_v2(
        ranges, angles, speed=6.3, heading_error=0.0,
        cross_track_error=0.0, curvatures=(0.1, 0.2, -0.3),
        target_speeds=(5.0, 4.0, 6.0), baseline_steering=0.25,
        speed_scale=6.3,
    )
    assert obs.shape == (OBSERVATION_DIM_V2,)
    assert obs.dtype == np.float32
    assert obs[18] == pytest.approx(1.0)
    assert obs[21] == pytest.approx(0.05)
    assert obs[23] == pytest.approx(-0.15)
    assert obs[24] == pytest.approx(5.0 / 6.3)


def test_v2_actor_loads_and_rejects_wrong_shape(tmp_path):
    weights = dict(
        format_version=np.array(2), observation_version=np.array(2),
        w0=np.zeros((64, OBSERVATION_DIM_V2), np.float32),
        b0=np.zeros(64, np.float32),
        w1=np.zeros((64, 64), np.float32), b1=np.zeros(64, np.float32),
        w2=np.zeros((2, 64), np.float32), b2=np.zeros(2, np.float32),
    )
    path = tmp_path / "v2.npz"
    np.savez(path, **weights)
    policy = ResidualPolicy.load(path)
    assert policy.observation_version == 2
    assert policy.predict(np.zeros(OBSERVATION_DIM_V2, np.float32)) == (0.0, 1.0)
    with pytest.raises(ValueError):
        policy.predict(np.zeros(24, np.float32))
    weights["w0"] = np.zeros((64, 24), np.float32)
    np.savez(path, **weights)
    with pytest.raises(ValueError):
        ResidualPolicy.load(path)
