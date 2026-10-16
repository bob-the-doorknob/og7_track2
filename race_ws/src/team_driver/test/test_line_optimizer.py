"""Map coordinates and whole-link clearance for offline racing lines."""

import numpy as np
import pytest
from PIL import Image

from training.track_map import TrackMap
from training.optimize_line import generate_line, predicted_time
from team_driver.vehicle_limits import VehicleLimits


@pytest.fixture
def small_map(tmp_path):
    pixels = np.full((8, 8), 254, dtype=np.uint8)
    pixels[3, 4] = 0
    pixels[3, 5] = 205
    Image.fromarray(pixels).save(tmp_path / "test.pgm")
    (tmp_path / "test.yaml").write_text(
        "image: test.pgm\nresolution: 1.0\norigin: [10.0, 20.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    return TrackMap(tmp_path / "test.yaml")


def test_map_world_coordinates_and_unknown_are_not_drivable(small_map):
    assert small_map.is_free(10.5, 20.5)
    assert not small_map.is_free(14.5, 24.5)  # occupied, image row 3
    assert not small_map.is_free(15.5, 24.5)  # unknown, image row 3
    assert not small_map.is_free(9.5, 20.5)
    assert not small_map.is_free(10.5, 28.5)


def test_segment_check_catches_wall_between_free_endpoints(small_map):
    assert small_map.is_free(13.5, 24.5)
    assert small_map.is_free(16.5, 24.5)
    assert not small_map.segment_has_clearance((13.5, 24.5), (16.5, 24.5), 0.1)


def test_clearance_checks_entire_link_and_out_of_bounds(small_map):
    assert small_map.segment_has_clearance((11.5, 21.5), (12.5, 21.5), 0.1)
    assert not small_map.segment_has_clearance((11.5, 21.5), (12.5, 21.5), 2.0)
    assert not small_map.segment_has_clearance((10.5, 20.5), (8.0, 20.5), 0.1)


def test_optimizer_returns_only_a_valid_improved_closed_line():
    class OpenMap:
        def line_clearance(self, points):
            return 10.0

    angles = np.linspace(0, 2 * np.pi, 32, endpoint=False)
    center = np.column_stack((5 * np.cos(angles), 5 * np.sin(angles)))
    limits = VehicleLimits(8.0, 3.0, 3.0, 3.0, 0.0)
    result = generate_line(center, OpenMap(), limits, 0.3,
                           initial=center, max_offset_m=0.5, passes=1)
    assert result.shape == center.shape
    assert np.max(np.abs(np.linalg.norm(result, axis=1) - 5.0)) <= 0.51
    assert predicted_time(result, limits) < predicted_time(center, limits)
