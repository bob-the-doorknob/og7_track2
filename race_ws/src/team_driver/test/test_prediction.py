import math

import pytest
import team_driver.pure_pursuit as pursuit


def test_predict_pose_follows_measured_constant_turn():
    assert hasattr(pursuit, 'predict_pose'), 'steering prediction helper is missing'
    position, yaw = pursuit.predict_pose((1., 2.), .3, 4., .8, .1)
    expected_x = 1 + 4/.8*(math.sin(.38)-math.sin(.3))
    expected_y = 2 + 4/.8*(math.cos(.3)-math.cos(.38))
    assert position == pytest.approx((expected_x, expected_y))
    assert yaw == pytest.approx(.38)


def test_zero_prediction_preserves_pose_and_straight_prediction_is_finite():
    assert hasattr(pursuit, 'predict_pose'), 'steering prediction helper is missing'
    assert pursuit.predict_pose((1., 2.), .3, 4., .8, 0.) == ((1., 2.), .3)
    position, yaw = pursuit.predict_pose((1., 2.), 0., 4., 0., .1)
    assert position == pytest.approx((1.4, 2.))
    assert yaw == 0.


def test_motion_interval_tracks_simulation_progress_not_wall_pacing():
    assert hasattr(pursuit, 'motion_interval'), 'motion interval estimator is missing'
    assert pursuit.motion_interval((0., 0.), (.25, 0.), 5., 5.) == pytest.approx(.05)
    assert pursuit.motion_interval((0., 0.), (.055, 0.), 5., 5.) == pytest.approx(.011)
    assert pursuit.motion_interval((0., 0.), (10., 0.), 5., 5.) is None
    assert pursuit.motion_interval((0., 0.), (0., 0.), 0., 0.) is None


def test_extra_lookahead_only_raises_the_high_speed_ceiling():
    import numpy as np
    path = pursuit.PurePursuit(np.asarray(((0.,0.),(10.,0.),(10.,10.),(0.,10.))),
        lookahead_min=.55, lookahead_max=1.5, lookahead_speed_gain=.25)
    assert path.command((1.,0.), 0., 5., lookahead_extra=.3)[1] == pytest.approx(1.8)
    assert path.command((1.,0.), 0., 2., lookahead_extra=.3)[1] == pytest.approx(1.05)
    assert path.command((1.,0.), 0., 5.)[1] == 1.5
