import math
import unittest

from orange_autonomy.local_planner import (LoopMissionTracker, PlanDecision,
                                           PlannerConfig, ScanPoint,
                                           choose_local_heading,
                                           command_from_decision,
                                           lane_target_heading,
                                           scan_points_from_ranges)


class LocalPlannerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = PlannerConfig()

    def test_scan_conversion_filters_invalid_measurements(self) -> None:
        points = scan_points_from_ranges(
            [float("nan"), float("inf"), 0.05, 1.5, 31.0],
            angle_min=-0.2,
            angle_increment=0.1,
            range_min=0.1,
            range_max=30.0,
        )
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0].angle_rad, 0.1)
        self.assertAlmostEqual(points[0].distance_m, 1.5)

    def test_lane_target_steers_toward_offset_center(self) -> None:
        left = lane_target_heading(1.5, 0.5, 0.0, self.config)
        right = lane_target_heading(1.5, -0.5, 0.0, self.config)
        self.assertGreater(left, 0.0)
        self.assertLess(right, 0.0)

    def test_clear_route_tracks_lane_heading(self) -> None:
        decision = choose_local_heading([], 0.2, 0.0, self.config)
        self.assertFalse(decision.blocked)
        candidate_step = (
            2.0 * self.config.max_heading_rad /
            (self.config.candidate_count - 1)
        )
        self.assertLess(
            abs(decision.selected_heading_rad - 0.2), candidate_step)
        self.assertEqual(decision.mode, "tracking")

    def test_center_obstacle_selects_a_side_route(self) -> None:
        obstacle = [ScanPoint(angle, 1.0) for angle in (-0.1, 0.0, 0.1)]
        decision = choose_local_heading(obstacle, 0.0, 0.0, self.config)
        self.assertFalse(decision.blocked)
        self.assertGreater(abs(decision.selected_heading_rad), 0.2)
        self.assertEqual(decision.mode, "avoiding")

    def test_selected_route_keeps_robot_corridor_inside_lane(self) -> None:
        obstacle = [ScanPoint(angle, 0.9) for angle in (-0.1, 0.0, 0.1)]
        decision = choose_local_heading(obstacle, 0.0, 0.0, self.config)
        selected_y = self.config.lane_constraint_horizon_m * math.sin(
            decision.selected_heading_rad
        )
        usable_half_width = (
            0.5 * self.config.nominal_lane_width_m
            - self.config.corridor_half_width_m
        )
        self.assertLessEqual(abs(selected_y), usable_half_width)

    def test_near_forward_obstacle_rotates_before_advancing(self) -> None:
        decision = PlanDecision(
            desired_heading_rad=0.0,
            selected_heading_rad=0.6,
            selected_clearance_m=4.0,
            forward_clearance_m=0.5,
            score=1.0,
            blocked=False,
            mode="avoiding",
        )
        command = command_from_decision(decision, 1.0, self.config)
        self.assertEqual(command.linear_mps, 0.0)
        self.assertGreater(command.angular_rps, 0.0)

    def test_speed_reduces_with_confidence(self) -> None:
        decision = PlanDecision(0.0, 0.0, 4.0, 4.0, 0.0, False, "tracking")
        confident = command_from_decision(decision, 1.0, self.config)
        uncertain = command_from_decision(decision, 0.2, self.config)
        self.assertGreater(confident.linear_mps, uncertain.linear_mps)

    def test_blocked_route_commands_rotation_only(self) -> None:
        obstacles = [
            ScanPoint(angle, 0.4)
            for angle in (
                -self.config.max_heading_rad,
                -0.4,
                0.0,
                0.4,
                self.config.max_heading_rad,
            )
        ]
        decision = choose_local_heading(obstacles, 0.0, 0.0, self.config)
        command = command_from_decision(decision, 1.0, self.config)
        self.assertTrue(decision.blocked)
        self.assertEqual(command.linear_mps, 0.0)
        self.assertGreaterEqual(abs(command.angular_rps),
                                self.config.blocked_turn_speed_rps)


class MissionTrackerTest(unittest.TestCase):
    def test_closed_loop_finishes_only_after_departure_and_distance(self) -> None:
        tracker = LoopMissionTracker(8.0, 3.0, 1.0, 2.0)
        progress = tracker.update(0.0, 0.0)
        self.assertFalse(progress.finished)
        for x_m, y_m in (
            (2.0, 0.0),
            (4.0, 0.0),
            (4.0, 2.0),
            (2.0, 2.0),
            (0.5, 0.5),
        ):
            progress = tracker.update(x_m, y_m)
        self.assertTrue(progress.departed_start)
        self.assertTrue(progress.finished)

    def test_odometry_jump_is_not_counted(self) -> None:
        tracker = LoopMissionTracker(10.0, 3.0, 1.0, 1.0)
        tracker.update(0.0, 0.0)
        progress = tracker.update(20.0, 20.0)
        self.assertTrue(math.isclose(progress.distance_travelled_m, 0.0))


if __name__ == "__main__":
    unittest.main()
