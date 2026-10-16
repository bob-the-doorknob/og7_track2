import importlib.util
import math

import pytest


def test_response_lag_recovers_three_sample_delay():
    assert importlib.util.find_spec('training.analyze_headless_pace') is not None
    from training.analyze_headless_pace import response_lag
    rows = []
    def command(t): return .12*math.sin(t*2.3)+.08*math.sin(t*5.7)
    for i in range(500):
        t=i*.05
        rows.append(dict(elapsed_s=t, speed_mps=4., steering=command(t),
            yaw_rate_radps=4*math.tan(command(t-.15)*.5236)/.324*.97))
    result = response_lag(rows)
    assert result['lag_wall_s'] == pytest.approx(.15, abs=.005)
    assert result['gain'] == pytest.approx(.97, abs=.02)
