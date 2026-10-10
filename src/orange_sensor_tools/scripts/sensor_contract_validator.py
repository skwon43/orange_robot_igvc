#!/usr/bin/env python3
"""Validate the externally visible ROS 2 sensor and frame contract."""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener


def merge_profile(document: dict[str, Any], profile_name: str) -> dict[str, Any]:
    profiles = document.get("profiles", {})
    if profile_name not in profiles:
        choices = ", ".join(sorted(profiles))
        raise ValueError(f"Unknown profile {profile_name!r}; choose one of: {choices}")

    profile = profiles[profile_name]
    base_name = profile.get("extends")
    if not base_name:
        return {
            "interfaces": list(profile.get("interfaces", [])),
            "required_transforms": list(profile.get("required_transforms", [])),
        }

    base = merge_profile(document, base_name)
    interfaces = {item["name"]: item for item in base["interfaces"]}
    for item in profile.get("interfaces", []):
        interfaces[item["name"]] = item

    transforms = list(base["required_transforms"])
    transforms.extend(profile.get("required_transforms", []))
    return {"interfaces": list(interfaces.values()), "required_transforms": transforms}


class SensorContractValidator(Node):
    def __init__(self) -> None:
        super().__init__("sensor_contract_validator")

        default_contract = str(
            Path(get_package_share_directory("orange_sensor_tools"))
            / "config"
            / "sensor_interface_contract.yaml"
        )
        self.declare_parameter("contract_file", default_contract)
        self.declare_parameter("profile", "simulation_baseline")
        self.declare_parameter("validation_window_sec", 10.0)
        self.declare_parameter("report_path", "")

        contract_path = Path(
            self.get_parameter("contract_file").get_parameter_value().string_value
        )
        profile_name = self.get_parameter("profile").get_parameter_value().string_value
        self.report_path = Path(
            self.get_parameter("report_path").get_parameter_value().string_value
        )
        self.window_sec = self.get_parameter(
            "validation_window_sec"
        ).get_parameter_value().double_value
        if self.window_sec <= 0.0:
            raise ValueError("validation_window_sec must be positive")

        with contract_path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        self.profile = merge_profile(document, profile_name)
        self.profile_name = profile_name
        self.contract_path = contract_path
        self.start_ns = time.monotonic_ns()
        self.completed = False
        self.contract_subscriptions = []
        self.interface_states: dict[str, dict[str, Any]] = {}

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        for interface in self.profile["interfaces"]:
            name = interface["name"]
            message_type = get_message(interface["message_type"])
            state = {
                "contract": interface,
                "sample_count": 0,
                "first_arrival_ns": None,
                "last_arrival_ns": None,
                "max_gap_sec": 0.0,
                "estimated_missed_samples": 0,
                "frame_ids": set(),
                "child_frame_ids": set(),
                "dimensions": set(),
                "errors": [],
            }
            self.interface_states[name] = state
            self.contract_subscriptions.append(
                self.create_subscription(
                    message_type,
                    interface["topic"],
                    self._make_callback(name),
                    qos,
                )
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.timer = self.create_timer(self.window_sec, self._finish_window)
        self.get_logger().info(
            f"Validating profile '{self.profile_name}' for {self.window_sec:.1f} s "
            f"using {self.contract_path}"
        )

    def _make_callback(self, name: str):
        def callback(message: Any) -> None:
            state = self.interface_states[name]
            now_ns = time.monotonic_ns()
            previous_ns = state["last_arrival_ns"]
            if previous_ns is not None:
                gap_sec = (now_ns - previous_ns) / 1_000_000_000.0
                state["max_gap_sec"] = max(state["max_gap_sec"], gap_sec)
                target_hz = state["contract"].get("target_hz")
                if target_hz and gap_sec > 0.0:
                    state["estimated_missed_samples"] += max(
                        0, round(gap_sec * float(target_hz)) - 1
                    )
            else:
                state["first_arrival_ns"] = now_ns
            state["last_arrival_ns"] = now_ns
            state["sample_count"] += 1

            header = getattr(message, "header", None)
            if header is not None:
                state["frame_ids"].add(header.frame_id)
            child_frame = getattr(message, "child_frame_id", None)
            if child_frame is not None:
                state["child_frame_ids"].add(child_frame)
            if hasattr(message, "width") and hasattr(message, "height"):
                state["dimensions"].add((message.width, message.height))

        return callback

    def _finish_window(self) -> None:
        self.completed = True
        self.timer.cancel()

    def _observed_hz(self, state: dict[str, Any]) -> float:
        first_ns = state["first_arrival_ns"]
        last_ns = state["last_arrival_ns"]
        samples = state["sample_count"]
        if first_ns is None or last_ns is None or samples < 2 or last_ns <= first_ns:
            return 0.0
        return (samples - 1) / ((last_ns - first_ns) / 1_000_000_000.0)

    def _validate_interface(
        self, state: dict[str, Any], graph: dict[str, list[str]]
    ) -> dict[str, Any]:
        contract = state["contract"]
        errors = list(state["errors"])
        topic = contract["topic"]
        expected_type = contract["message_type"]
        observed_types = graph.get(topic, [])
        if expected_type not in observed_types:
            errors.append(
                f"topic type mismatch: expected {expected_type}, observed {observed_types}"
            )

        validate_samples = contract.get(
            "validate_samples", contract.get("direction") != "command"
        )
        required = contract.get("required", True)
        samples = state["sample_count"]
        observed_hz = self._observed_hz(state)
        if required and validate_samples and samples == 0:
            errors.append("no messages received")
        if validate_samples and samples > 0:
            expected_frame = contract.get("expected_frame_id")
            if expected_frame and state["frame_ids"] != {expected_frame}:
                errors.append(
                    f"frame_id mismatch: expected {expected_frame}, "
                    f"observed {sorted(state['frame_ids'])}"
                )
            expected_child = contract.get("expected_child_frame_id")
            if expected_child and state["child_frame_ids"] != {expected_child}:
                errors.append(
                    f"child_frame_id mismatch: expected {expected_child}, "
                    f"observed {sorted(state['child_frame_ids'])}"
                )
            expected_dimensions = (
                contract.get("expected_width"),
                contract.get("expected_height"),
            )
            if all(value is not None for value in expected_dimensions):
                if state["dimensions"] != {expected_dimensions}:
                    errors.append(
                        f"image dimensions mismatch: expected {expected_dimensions}, "
                        f"observed {sorted(state['dimensions'])}"
                    )
            min_hz = contract.get("min_hz")
            if min_hz is not None and observed_hz < float(min_hz):
                errors.append(
                    f"rate below contract: {observed_hz:.2f} Hz < {float(min_hz):.2f} Hz"
                )

        return {
            "name": contract["name"],
            "topic": topic,
            "expected_type": expected_type,
            "observed_types": observed_types,
            "samples": samples,
            "observed_hz": round(observed_hz, 3),
            "max_gap_sec": round(state["max_gap_sec"], 4),
            "estimated_missed_samples": state["estimated_missed_samples"],
            "observed_frame_ids": sorted(state["frame_ids"]),
            "observed_child_frame_ids": sorted(state["child_frame_ids"]),
            "observed_dimensions": sorted(state["dimensions"]),
            "errors": errors,
            "passed": not errors,
        }

    def build_report(self) -> tuple[dict[str, Any], int]:
        graph = {
            topic: sorted(types)
            for topic, types in self.get_topic_names_and_types()
        }
        interfaces = [
            self._validate_interface(state, graph)
            for state in self.interface_states.values()
        ]
        transforms = []
        for edge in self.profile["required_transforms"]:
            available = self.tf_buffer.can_transform(
                edge["target_frame"],
                edge["source_frame"],
                Time(),
                timeout=Duration(seconds=0.0),
            )
            transforms.append({**edge, "passed": available})

        passed = all(item["passed"] for item in interfaces) and all(
            item["passed"] for item in transforms
        )
        report = {
            "status": "passed" if passed else "failed",
            "profile": self.profile_name,
            "contract_file": str(self.contract_path),
            "validation_window_sec": self.window_sec,
            "interfaces": interfaces,
            "transforms": transforms,
        }
        if self.report_path:
            self.report_path.parent.mkdir(parents=True, exist_ok=True)
            self.report_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            self.get_logger().info(f"Wrote validation report to {self.report_path}")

        failures = [item["name"] for item in interfaces if not item["passed"]]
        failures.extend(
            f"{item['target_frame']}<-{item['source_frame']}"
            for item in transforms
            if not item["passed"]
        )
        if failures:
            self.get_logger().error("Contract validation failed: " + ", ".join(failures))
        else:
            self.get_logger().info("Contract validation passed")
        return report, 0 if passed else 1


def main() -> int:
    rclpy.init()
    node: SensorContractValidator | None = None
    try:
        node = SensorContractValidator()
        while rclpy.ok() and not node.completed:
            rclpy.spin_once(node, timeout_sec=0.2)
        _, exit_code = node.build_report()
        return exit_code
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"sensor_contract_validator: {error}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
