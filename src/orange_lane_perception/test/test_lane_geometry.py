import math
import unittest

from orange_lane_perception.lane_geometry import (estimate_lane_center,
                                                  intersect_ground_plane,
                                                  rotate_vector)


class LaneGeometryTest(unittest.TestCase):
    def test_identity_rotation_preserves_vector(self) -> None:
        self.assertEqual(
            rotate_vector((1.0, -2.0, 3.0), (0.0, 0.0, 0.0, 1.0)),
            (1.0, -2.0, 3.0),
        )

    def test_ground_intersection_requires_forward_ray(self) -> None:
        self.assertEqual(
            intersect_ground_plane((0.0, 0.0, 1.0), (1.0, 0.0, -1.0)),
            (1.0, 0.0),
        )
        self.assertIsNone(intersect_ground_plane(
            (0.0, 0.0, 1.0), (1.0, 0.0, 1.0)))

    def test_lane_center_matches_parallel_boundaries(self) -> None:
        left = [(x, 2.0 + 0.1 * x) for x in (0.5, 1.0, 1.5, 2.0, 2.5)]
        right = [(x, -2.0 + 0.1 * x) for x in (0.5, 1.0, 1.5, 2.0, 2.5)]
        estimate = estimate_lane_center(
            left,
            right,
            lookahead_m=1.5,
            min_lane_width_m=3.0,
            max_lane_width_m=5.0,
            min_points_per_side=3,
        )
        self.assertIsNotNone(estimate)
        assert estimate is not None
        self.assertAlmostEqual(estimate.x_m, 1.5)
        self.assertAlmostEqual(estimate.y_m, 0.15)
        self.assertAlmostEqual(estimate.lane_width_m, 4.0)
        self.assertAlmostEqual(estimate.heading_rad, math.atan(0.1))

    def test_rejects_implausible_lane_width(self) -> None:
        left = [(0.5, 4.0), (1.0, 4.0), (1.5, 4.0)]
        right = [(0.5, -4.0), (1.0, -4.0), (1.5, -4.0)]
        self.assertIsNone(
            estimate_lane_center(left, right, 1.0, 3.0,
                                 5.0, min_points_per_side=3)
        )


if __name__ == "__main__":
    unittest.main()
