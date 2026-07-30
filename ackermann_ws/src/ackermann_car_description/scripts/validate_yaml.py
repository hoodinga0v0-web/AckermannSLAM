#!/usr/bin/env python3
"""Strictly validate YAML files, including duplicate keys and generated contracts."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import sys
from typing import Any

import yaml

from normalize_meshes import (
    PipelineError,
    UniqueKeyLoader,
    load_geometry,
    require_number,
    validate_geometry_schema,
)


PLACEHOLDER = re.compile(
    r"(TODO_CAD|REQUIRED|MODELED_WHEELBASE_M|MODELED_STEERING_AXIS_TRACK_M|"
    r"REAR_TRACK_M|REAR_EFFECTIVE_RADIUS_M|STEERING_POSITION_GAIN|SAFE_)"
)

GUARD_PARAMETERS = {
    "use_sim_time",
    "rolling_wheelbase",
    "steering_track_width",
    "traction_track_width",
    "traction_wheels_radius",
    "r_min_left",
    "r_min_right",
    "rear_left_wheel_velocity",
    "rear_right_wheel_velocity",
    "rear_left_wheel_acceleration",
    "rear_right_wheel_acceleration",
    "front_left_steering_lower",
    "front_left_steering_upper",
    "front_left_steering_velocity",
    "front_right_steering_lower",
    "front_right_steering_upper",
    "front_right_steering_velocity",
    "publish_rate",
    "command_timeout",
    "max_input_age",
    "future_tolerance",
    "zero_linear_epsilon",
    "max_projection_iterations",
    "input_topic",
    "output_topic",
    "base_frame_id",
}


def finite_and_placeholder_free(value: Any, label: str) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            finite_and_placeholder_free(child, f"{label}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            finite_and_placeholder_free(child, f"{label}[{index}]")
    elif isinstance(value, float) and not math.isfinite(value):
        raise PipelineError(f"{label} contains a non-finite number")
    elif isinstance(value, str) and PLACEHOLDER.search(value):
        raise PipelineError(f"{label} contains unresolved placeholder {value!r}")


def load_yaml(path: Path) -> Any:
    try:
        raw = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise PipelineError(f"cannot read {path}: {exc}") from exc
    try:
        document = yaml.load(raw, Loader=UniqueKeyLoader)
    except yaml.YAMLError as exc:
        raise PipelineError(f"invalid YAML in {path}: {exc}") from exc
    finite_and_placeholder_free(document, path.name)
    return document


def validate_bridge(document: Any, path: Path) -> None:
    if not isinstance(document, list) or not document:
        raise PipelineError(f"{path} root must be a non-empty sequence")
    required = {
        "ros_topic_name",
        "gz_topic_name",
        "ros_type_name",
        "gz_type_name",
        "direction",
    }
    for index, entry in enumerate(document):
        if not isinstance(entry, dict):
            raise PipelineError(f"{path}[{index}] must be a mapping")
        missing = required - set(entry)
        if missing:
            raise PipelineError(f"{path}[{index}] missing keys {sorted(missing)}")


def parameters(document: dict[str, Any], node_name: str, path: Path) -> dict[str, Any]:
    node = document.get(node_name)
    if not isinstance(node, dict):
        raise PipelineError(f"{path} must contain mapping {node_name}")
    result = node.get("ros__parameters")
    if not isinstance(result, dict):
        raise PipelineError(f"{path}:{node_name}.ros__parameters must be a mapping")
    return result


def validate_controllers(document: dict[str, Any], path: Path) -> None:
    manager = parameters(document, "controller_manager", path)
    controller = parameters(document, "ackermann_steering_controller", path)
    if manager.get("enforce_command_limits") is not True:
        raise PipelineError(f"{path} must enable enforce_command_limits")
    if manager.get("use_sim_time") is not True or controller.get("use_sim_time") is not True:
        raise PipelineError(f"{path} must use simulation time")
    expected_traction = ["rear_right_wheel_joint", "rear_left_wheel_joint"]
    expected_steering = [
        "front_right_steering_joint",
        "front_left_steering_joint",
    ]
    if controller.get("traction_joints_names") != expected_traction:
        raise PipelineError(f"{path} traction joint order must be right, left")
    if controller.get("steering_joints_names") != expected_steering:
        raise PipelineError(f"{path} steering joint order must be right, left")
    for name in (
        "wheelbase",
        "traction_track_width",
        "steering_track_width",
        "traction_wheels_radius",
        "reference_timeout",
    ):
        if require_number(controller.get(name), f"{path.name}.{name}") <= 0.0:
            raise PipelineError(f"{path}:{name} must be positive")


def validate_guard(document: dict[str, Any], path: Path) -> None:
    guard = parameters(document, "ackermann_command_guard", path)
    missing = GUARD_PARAMETERS - set(guard)
    unexpected = set(guard) - GUARD_PARAMETERS
    if missing or unexpected:
        raise PipelineError(
            f"{path} guard parameter contract mismatch; "
            f"missing={sorted(missing)}, unexpected={sorted(unexpected)}"
        )
    if guard["use_sim_time"] is not True:
        raise PipelineError(f"{path} guard must use simulation time")
    if guard["input_topic"] != "/cmd_vel_raw" or guard["output_topic"] != "/cmd_vel":
        raise PipelineError(f"{path} guard topics do not match the command chain")
    if guard["base_frame_id"] != "base_footprint":
        raise PipelineError(f"{path} guard base_frame_id must be base_footprint")
    for name in GUARD_PARAMETERS - {
        "use_sim_time",
        "input_topic",
        "output_topic",
        "base_frame_id",
        "front_left_steering_lower",
        "front_right_steering_lower",
    }:
        value = require_number(guard.get(name), f"{path.name}.{name}")
        if value <= 0.0:
            raise PipelineError(f"{path}:{name} must be positive")
    for side in ("left", "right"):
        lower = require_number(
            guard[f"front_{side}_steering_lower"],
            f"{path.name}.front_{side}_steering_lower",
        )
        upper = require_number(
            guard[f"front_{side}_steering_upper"],
            f"{path.name}.front_{side}_steering_upper",
        )
        if not lower < 0.0 < upper:
            raise PipelineError(f"{path} {side} steering limits must straddle zero")


def validate_consistency(documents: dict[str, dict[str, Any]]) -> None:
    geometry = documents.get("vehicle_geometry.yaml")
    controllers = documents.get("controllers.yaml")
    guard = documents.get("command_guard.yaml")
    if geometry is None or controllers is None or guard is None:
        return
    model = geometry["geometry"]
    controller = parameters(
        controllers, "ackermann_steering_controller", Path("controllers.yaml")
    )
    guard_params = parameters(
        guard, "ackermann_command_guard", Path("command_guard.yaml")
    )
    controller_comparisons = {
        "wheelbase": model["rolling_wheelbase"],
        "traction_track_width": model["traction_track_width"],
        "steering_track_width": model["steering_track_width"],
        "traction_wheels_radius": model["rear_wheel_radius"],
    }
    for name, expected in controller_comparisons.items():
        if abs(float(controller[name]) - float(expected)) > 1.0e-12:
            raise PipelineError(f"controllers.yaml {name} is stale")
    guard_comparisons = {
        "rolling_wheelbase": model["rolling_wheelbase"],
        "traction_track_width": model["traction_track_width"],
        "steering_track_width": model["steering_track_width"],
        "traction_wheels_radius": model["rear_wheel_radius"],
    }
    for name, expected in guard_comparisons.items():
        if abs(float(guard_params[name]) - float(expected)) > 1.0e-12:
            raise PipelineError(f"command_guard.yaml {name} is stale")


def collect_files(paths: list[Path]) -> list[Path]:
    files: set[Path] = set()
    for path in paths:
        if path.is_dir():
            files.update(path.rglob("*.yaml"))
            files.update(path.rglob("*.yml"))
        elif path.is_file():
            if path.suffix.lower() not in {".yaml", ".yml"}:
                raise PipelineError(f"{path} is not a YAML file")
            files.add(path)
        else:
            raise PipelineError(f"{path} does not exist")
    if not files:
        raise PipelineError("no YAML files found")
    return sorted(files)


def parse_arguments() -> argparse.Namespace:
    package_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "paths",
        type=Path,
        nargs="*",
        default=[package_root / "config"],
        help="YAML files or directories (default: package config directory)",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        files = collect_files(arguments.paths)
        documents: dict[str, dict[str, Any]] = {}
        for path in files:
            document = load_yaml(path)
            if path.name == "bridge.yaml":
                validate_bridge(document, path)
                continue
            if not isinstance(document, dict):
                raise PipelineError(f"{path} root must be a mapping")
            if path.name == "vehicle_geometry.yaml":
                geometry, _ = load_geometry(path)
                validate_geometry_schema(geometry)
            elif path.name == "controllers.yaml":
                validate_controllers(document, path)
            elif path.name == "command_guard.yaml":
                validate_guard(document, path)
            documents[path.name] = document
        validate_consistency(documents)
        print(f"validated {len(files)} YAML file(s)")
        return 0
    except (PipelineError, OSError, ValueError, KeyError) as exc:
        print(f"validate_yaml.py: error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
