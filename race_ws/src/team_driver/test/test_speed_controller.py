"""The fast controller can use full throttle and actively brake."""

from team_driver.speed_controller import SpeedController, control_dt


def test_nominal_control_dt_is_independent_of_window_pacing():
    assert control_dt("nominal", 10.2, 10.0) == 0.025
    assert control_dt("nominal", 10.02, 10.0) == 0.025
    assert abs(control_dt("wall", 10.2, 10.0) - 0.2) < 1e-8


def test_fast_controller_uses_full_throttle_when_far_below_target():
    controller = SpeedController(throttle_max=1.0, throttle_min=-1.0,
                                 active_braking=True)
    assert controller.command(target_speed=9.0, speed=0.0, dt=0.025) == 1.0


def test_fast_controller_brakes_before_a_slow_corner():
    controller = SpeedController(throttle_max=1.0, throttle_min=-1.0,
                                 active_braking=True)
    assert controller.command(target_speed=2.0, speed=4.0, dt=0.025) < 0.0


def test_legacy_controller_keeps_modest_actuator_range():
    controller = SpeedController()
    assert controller.command(target_speed=9.0, speed=0.0, dt=0.025) == 0.65


def test_approach_band_removes_full_throttle_near_target_without_delaying_brakes():
    controller = SpeedController(throttle_max=1.0, throttle_min=-1.0,
                                 active_braking=True, approach_band_mps=0.3)
    accelerating = controller.command(6.37, 6.378, 0.025)
    braking = controller.command(6.37, 6.447, 0.025)
    assert accelerating == 0.0
    assert braking < 0.0
    assert abs(accelerating - braking) < 0.05
    # A sudden safety target drop must retain the original braking strength.
    assert abs(controller.command(3.0, 6.0, 0.025) + 0.9) < 1e-12


def test_approach_band_tapers_acceleration_but_keeps_full_throttle_for_large_deficit():
    controller = SpeedController(throttle_max=1.0, throttle_min=-1.0,
                                 active_braking=True, approach_band_mps=0.3)
    assert controller.command(7.2, 4.0, 0.025) == 1.0
    assert 0 < controller.command(7.2, 7.18, 0.025) < 0.1
    assert controller.command(7.2, 7.2, 0.025) == 0.0


def test_approach_band_validation():
    import pytest

    for invalid in (-0.1, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            SpeedController(approach_band_mps=invalid)
