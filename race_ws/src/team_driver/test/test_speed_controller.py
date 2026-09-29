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
