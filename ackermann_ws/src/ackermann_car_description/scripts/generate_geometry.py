#!/usr/bin/env python3
"""Generate Xacro and controller configuration from canonical vehicle geometry."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Callable
from xml.sax.saxutils import escape as xml_escape

import yaml

from normalize_meshes import (
    PipelineError,
    UniqueKeyLoader,
    atomic_write,
    load_geometry,
    require_number,
    require_vector,
    sha256_bytes,
    validate_geometry_schema,
)


GENERATOR_ID = "ackermann_car_description/generate_geometry.py:v1"


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PipelineError(f"{label} must be a mapping")
    return value


def require_positive(value: Any, label: str) -> float:
    number = require_number(value, label)
    if number <= 0.0:
        raise PipelineError(f"{label} must be positive")
    return number


def duplicate_rejecting_json(data: bytes, label: str) -> dict[str, Any]:
    def object_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise PipelineError(f"duplicate JSON key {key!r} in {label}")
            result[key] = value
        return result

    try:
        parsed = json.loads(data.decode("utf-8"), object_pairs_hook=object_pairs)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PipelineError(f"invalid JSON in {label}: {exc}") from exc
    if not isinstance(parsed, dict):
        raise PipelineError(f"{label} root must be an object")
    return parsed


def validate_inertia(entry: dict[str, Any], label: str) -> dict[str, Any]:
    mass = require_positive(entry.get("mass"), f"{label}.mass")
    com = require_vector(entry.get("com_xyz"), 3, f"{label}.com_xyz")
    inertia = require_mapping(entry.get("inertia"), f"{label}.inertia")
    values = {
        name: require_number(inertia.get(name), f"{label}.inertia.{name}")
        for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    }
    if values["ixx"] <= 0.0:
        raise PipelineError(f"{label} inertia is not positive definite")
    leading_two = values["ixx"] * values["iyy"] - values["ixy"] ** 2
    determinant = (
        values["ixx"]
        * (values["iyy"] * values["izz"] - values["iyz"] ** 2)
        - values["ixy"]
        * (values["ixy"] * values["izz"] - values["iyz"] * values["ixz"])
        + values["ixz"]
        * (values["ixy"] * values["iyz"] - values["iyy"] * values["ixz"])
    )
    if leading_two <= 0.0 or determinant <= 0.0:
        raise PipelineError(f"{label} inertia is not positive definite")
    return {"mass": mass, "com_xyz": com, **values}


def validate_limit(
    limits: dict[str, Any], name: str, steering: bool
) -> dict[str, float]:
    entry = require_mapping(limits.get(name), f"limits.{name}")
    result = {
        key: require_positive(entry.get(key), f"limits.{name}.{key}")
        for key in (("velocity", "effort") if steering else ("velocity", "acceleration", "effort"))
    }
    if steering:
        lower = require_number(entry.get("lower"), f"limits.{name}.lower")
        upper = require_number(entry.get("upper"), f"limits.{name}.upper")
        if not lower < 0.0 < upper or lower >= upper:
            raise PipelineError(f"limits.{name} must straddle zero")
        result["lower"] = lower
        result["upper"] = upper
    return result


def steering_angles(
    curvature: float, wheelbase: float, steering_track: float
) -> tuple[float, float]:
    return (
        math.atan2(
            wheelbase * curvature, 1.0 - steering_track * curvature / 2.0
        ),
        math.atan2(
            wheelbase * curvature, 1.0 + steering_track * curvature / 2.0
        ),
    )


def minimum_turning_radius(
    direction: float,
    wheelbase: float,
    steering_track: float,
    left_limit: dict[str, float],
    right_limit: dict[str, float],
) -> float:
    def feasible(magnitude: float) -> bool:
        left, right = steering_angles(
            direction * magnitude, wheelbase, steering_track
        )
        return (
            left_limit["lower"] <= left <= left_limit["upper"]
            and right_limit["lower"] <= right <= right_limit["upper"]
        )

    if not feasible(0.0):
        raise PipelineError("zero curvature is outside steering joint limits")
    lower = 0.0
    upper = 1.0
    while feasible(upper) and upper < 1.0e6:
        lower = upper
        upper *= 2.0
    if feasible(upper):
        raise PipelineError("steering limits do not produce a finite turning radius")
    for _ in range(100):
        midpoint = (lower + upper) / 2.0
        if feasible(midpoint):
            lower = midpoint
        else:
            upper = midpoint
    if lower <= 0.0:
        raise PipelineError("computed non-positive feasible curvature")
    return 1.0 / lower


def finite_tree(value: Any, label: str = "root") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            finite_tree(child, f"{label}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            finite_tree(child, f"{label}[{index}]")
    elif isinstance(value, float) and not math.isfinite(value):
        raise PipelineError(f"{label} is not finite")


def format_number(value: float | int) -> str:
    number = float(value)
    if number == 0.0:
        return "0"
    return f"{number:.12g}"


def format_vector(value: list[float]) -> str:
    return " ".join(format_number(component) for component in value)


def xacro_property(name: str, value: Any) -> str:
    if isinstance(value, bool):
        rendered = "true" if value else "false"
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        rendered = format_number(value)
    elif isinstance(value, list):
        rendered = format_vector([float(component) for component in value])
    else:
        rendered = str(value)
    return (
        f'  <xacro:property name="{xml_escape(name)}" '
        f'value="{xml_escape(rendered)}"/>\n'
    )


def yaml_bytes(header: list[str], document: dict[str, Any]) -> bytes:
    finite_tree(document)
    comments = "".join(f"# {line}\n" for line in header)
    dumped = yaml.safe_dump(
        document,
        sort_keys=False,
        default_flow_style=False,
        allow_unicode=True,
    )
    return (comments + dumped).encode("utf-8")


def validate_manifest(
    manifest_path: Path,
    geometry_sha256: str,
    geometry: dict[str, Any],
    package_root: Path,
) -> tuple[dict[str, Any], bytes]:
    try:
        raw = manifest_path.read_bytes()
    except OSError as exc:
        raise PipelineError(
            f"cannot read {manifest_path}; run normalize_meshes.py first: {exc}"
        ) from exc
    manifest = duplicate_rejecting_json(raw, str(manifest_path))
    if manifest.get("canonical_geometry_sha256") != geometry_sha256:
        raise PipelineError("mesh manifest was generated from stale canonical geometry")
    if manifest.get("expected_input_count") != 10 or manifest.get("expected_output_count") != 9:
        raise PipelineError("mesh manifest does not describe the required 10-to-9 mapping")
    records = manifest.get("output_meshes")
    if not isinstance(records, list) or len(records) != 9:
        raise PipelineError("mesh manifest output_meshes must contain 9 records")
    expected_targets = {
        entry["target"] for entry in geometry["meshes"] if entry.get("include")
    }
    manifest_targets = {
        record.get("target") for record in records if isinstance(record, dict)
    }
    if manifest_targets != expected_targets:
        raise PipelineError("mesh manifest targets do not match canonical geometry")
    for record in records:
        if not isinstance(record, dict):
            raise PipelineError("invalid mesh manifest output record")
        target = record["target"]
        path = package_root / "meshes" / target
        try:
            actual_hash = sha256_bytes(path.read_bytes())
        except OSError as exc:
            raise PipelineError(f"cannot read normalized mesh {path}: {exc}") from exc
        if actual_hash != record.get("target_sha256"):
            raise PipelineError(f"normalized mesh {target} is stale")
    return manifest, raw


def build_documents(
    geometry_path: Path,
    manifest_path: Path,
    package_root: Path,
) -> dict[Path, bytes]:
    geometry, geometry_raw = load_geometry(geometry_path)
    validate_geometry_schema(geometry)
    geometry_hash = sha256_bytes(geometry_raw)
    manifest, manifest_raw = validate_manifest(
        manifest_path, geometry_hash, geometry, package_root
    )
    manifest_hash = sha256_bytes(manifest_raw)

    model_geometry = require_mapping(geometry.get("geometry"), "geometry")
    wheelbase = require_positive(
        model_geometry.get("rolling_wheelbase"), "geometry.rolling_wheelbase"
    )
    traction_track = require_positive(
        model_geometry.get("traction_track_width"),
        "geometry.traction_track_width",
    )
    steering_track = require_positive(
        model_geometry.get("steering_track_width"),
        "geometry.steering_track_width",
    )
    rear_radius = require_positive(
        model_geometry.get("rear_wheel_radius"), "geometry.rear_wheel_radius"
    )
    front_radius = require_positive(
        model_geometry.get("front_wheel_radius"), "geometry.front_wheel_radius"
    )
    rear_width = require_positive(
        model_geometry.get("rear_wheel_width"), "geometry.rear_wheel_width"
    )
    front_width = require_positive(
        model_geometry.get("front_wheel_width"), "geometry.front_wheel_width"
    )
    rear_height = require_positive(
        model_geometry.get("rear_axle_height"), "geometry.rear_axle_height"
    )
    front_height = require_positive(
        model_geometry.get("front_axle_height"), "geometry.front_axle_height"
    )
    if abs(rear_height - rear_radius) > 1.0e-9:
        raise PipelineError("rear axle height must equal provisional rear wheel radius")
    if abs(front_height - front_radius) > 1.0e-9:
        raise PipelineError("front axle height must equal provisional front wheel radius")

    limits = require_mapping(geometry.get("limits"), "limits")
    steering_left = validate_limit(limits, "front_left_steering", True)
    steering_right = validate_limit(limits, "front_right_steering", True)
    rear_left = validate_limit(limits, "rear_left_wheel", False)
    rear_right = validate_limit(limits, "rear_right_wheel", False)
    radius_left = minimum_turning_radius(
        1.0, wheelbase, steering_track, steering_left, steering_right
    )
    radius_right = minimum_turning_radius(
        -1.0, wheelbase, steering_track, steering_left, steering_right
    )

    control = require_mapping(geometry.get("control"), "control")
    update_rate = int(require_positive(control.get("update_rate"), "control.update_rate"))
    command_timeout = require_positive(
        control.get("command_timeout"), "control.command_timeout"
    )
    zero_epsilon = require_positive(
        control.get("zero_linear_epsilon"), "control.zero_linear_epsilon"
    )
    iterations = int(
        require_positive(
            control.get("max_projection_iterations"),
            "control.max_projection_iterations",
        )
    )
    gain_config = require_mapping(
        control.get("steering_position_gain"), "control.steering_position_gain"
    )
    error_bound = require_positive(
        gain_config.get("maximum_position_error"),
        "control.steering_position_gain.maximum_position_error",
    )
    safety_factor = require_positive(
        gain_config.get("safety_factor"),
        "control.steering_position_gain.safety_factor",
    )
    if safety_factor > 1.0:
        raise PipelineError("steering_position_gain.safety_factor must be <= 1")
    gain_cap = require_positive(
        gain_config.get("cap"), "control.steering_position_gain.cap"
    )
    steering_velocity = min(
        steering_left["velocity"], steering_right["velocity"]
    )
    steering_position_gain = min(
        gain_cap,
        safety_factor * steering_velocity / (error_bound * update_rate),
    )
    if steering_position_gain <= 0.0:
        raise PipelineError("computed steering_position_gain is not positive")

    lidar = require_mapping(geometry.get("lidar"), "lidar")
    lidar_body_xyz = require_vector(lidar.get("body_xyz"), 3, "lidar.body_xyz")
    lidar_body_rpy = require_vector(lidar.get("body_rpy"), 3, "lidar.body_rpy")
    lidar_ray_xyz = require_vector(lidar.get("ray_xyz"), 3, "lidar.ray_xyz")
    lidar_ray_rpy = require_vector(lidar.get("ray_rpy"), 3, "lidar.ray_rpy")
    collision = require_mapping(geometry.get("collision"), "collision")
    chassis_collision = require_mapping(collision.get("chassis"), "collision.chassis")
    chassis_collision_origin = require_vector(
        chassis_collision.get("origin_xyz"), 3, "collision.chassis.origin_xyz"
    )
    chassis_collision_size = require_vector(
        chassis_collision.get("size_xyz"), 3, "collision.chassis.size_xyz"
    )
    if any(component <= 0.0 for component in chassis_collision_size):
        raise PipelineError("collision.chassis.size_xyz must be positive")

    inertials_config = require_mapping(geometry.get("inertials"), "inertials")
    inertia_names = (
        "base",
        "front_left_steering",
        "front_right_steering",
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    )
    inertials = {
        name: validate_inertia(
            require_mapping(inertials_config.get(name), f"inertials.{name}"),
            f"inertials.{name}",
        )
        for name in inertia_names
    }

    coordinate = require_mapping(geometry.get("coordinate_system"), "coordinate_system")
    mesh_scale_value = require_positive(
        coordinate.get("mesh_export_unit"), "coordinate_system.mesh_export_unit"
    )
    mesh_scale = [mesh_scale_value, mesh_scale_value, mesh_scale_value]
    mesh_targets = {
        entry["source"]: entry["target"]
        for entry in geometry["meshes"]
        if entry.get("include")
    }
    expected_sources = {
        "chassis.stl",
        "exterior.stl",
        "LFWheel.stl",
        "RFWheel.stl",
        "LRWheel.stl",
        "RRWheel.stl",
        "LFKnuckle.stl",
        "RFKnuckle.stl",
        "LiDER.stl",
    }
    if set(mesh_targets) != expected_sources:
        raise PipelineError("canonical mesh mapping is incomplete")

    properties: list[tuple[str, Any]] = [
        ("geometry_status", geometry["status"]["classification"]),
        ("geometry_model", geometry["model"]),
        ("geometry_provisional", True),
        ("geometry_production_ready", False),
        ("geometry_sha256", geometry_hash),
        ("mesh_manifest_sha256", manifest_hash),
        ("mesh_scale", mesh_scale),
        ("rolling_wheelbase", wheelbase),
        ("traction_track_width", traction_track),
        ("steering_track_width", steering_track),
        ("rear_wheel_radius", rear_radius),
        ("front_wheel_radius", front_radius),
        ("rear_wheel_width", rear_width),
        ("front_wheel_width", front_width),
        ("rear_axle_height", rear_height),
        ("front_axle_height", front_height),
        ("steering_position_gain", steering_position_gain),
        ("minimum_turning_radius_left", radius_left),
        ("minimum_turning_radius_right", radius_right),
        ("use_knuckle_visual", bool(geometry["visual"]["use_knuckle_visual"])),
        ("chassis_collision_origin_xyz", chassis_collision_origin),
        ("chassis_collision_size_xyz", chassis_collision_size),
        ("lidar_body_x", lidar_body_xyz[0]),
        ("lidar_body_y", lidar_body_xyz[1]),
        ("lidar_body_z", lidar_body_xyz[2]),
        ("lidar_body_roll", lidar_body_rpy[0]),
        ("lidar_body_pitch", lidar_body_rpy[1]),
        ("lidar_body_yaw", lidar_body_rpy[2]),
        ("lidar_ray_x", lidar_ray_xyz[0]),
        ("lidar_ray_y", lidar_ray_xyz[1]),
        ("lidar_ray_z", lidar_ray_xyz[2]),
        ("lidar_ray_roll", lidar_ray_rpy[0]),
        ("lidar_ray_pitch", lidar_ray_rpy[1]),
        ("lidar_ray_yaw", lidar_ray_rpy[2]),
        ("chassis_mesh_file", mesh_targets["chassis.stl"]),
        ("exterior_mesh_file", mesh_targets["exterior.stl"]),
        ("front_left_wheel_mesh_file", mesh_targets["LFWheel.stl"]),
        ("front_right_wheel_mesh_file", mesh_targets["RFWheel.stl"]),
        ("rear_left_wheel_mesh_file", mesh_targets["LRWheel.stl"]),
        ("rear_right_wheel_mesh_file", mesh_targets["RRWheel.stl"]),
        ("front_left_knuckle_mesh_file", mesh_targets["LFKnuckle.stl"]),
        ("front_right_knuckle_mesh_file", mesh_targets["RFKnuckle.stl"]),
        ("lidar_body_mesh_file", mesh_targets["LiDER.stl"]),
        ("front_left_steering_lower", steering_left["lower"]),
        ("front_left_steering_upper", steering_left["upper"]),
        ("front_left_steering_velocity", steering_left["velocity"]),
        ("front_left_steering_effort", steering_left["effort"]),
        ("front_right_steering_lower", steering_right["lower"]),
        ("front_right_steering_upper", steering_right["upper"]),
        ("front_right_steering_velocity", steering_right["velocity"]),
        ("front_right_steering_effort", steering_right["effort"]),
        ("rear_left_wheel_velocity", rear_left["velocity"]),
        ("rear_left_wheel_acceleration", rear_left["acceleration"]),
        ("rear_left_wheel_effort", rear_left["effort"]),
        ("rear_right_wheel_velocity", rear_right["velocity"]),
        ("rear_right_wheel_acceleration", rear_right["acceleration"]),
        ("rear_right_wheel_effort", rear_right["effort"]),
        ("rear_left_wheel_origin_xyz", [0.0, traction_track / 2.0, 0.0]),
        ("rear_right_wheel_origin_xyz", [0.0, -traction_track / 2.0, 0.0]),
        (
            "front_left_steering_origin_xyz",
            [wheelbase, steering_track / 2.0, front_height - rear_height],
        ),
        (
            "front_right_steering_origin_xyz",
            [wheelbase, -steering_track / 2.0, front_height - rear_height],
        ),
    ]
    for name in inertia_names:
        inertia = inertials[name]
        properties.extend(
            [
                (f"{name}_mass", inertia["mass"]),
                (f"{name}_com_xyz", inertia["com_xyz"]),
                (f"{name}_ixx", inertia["ixx"]),
                (f"{name}_ixy", inertia["ixy"]),
                (f"{name}_ixz", inertia["ixz"]),
                (f"{name}_iyy", inertia["iyy"]),
                (f"{name}_iyz", inertia["iyz"]),
                (f"{name}_izz", inertia["izz"]),
            ]
        )

    xacro_lines = [
        '<?xml version="1.0"?>\n',
        f"<!-- GENERATED by {GENERATOR_ID}; do not edit. -->\n",
        f"<!-- canonical_geometry_sha256: {geometry_hash} -->\n",
        f"<!-- mesh_manifest_sha256: {manifest_hash} -->\n",
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n',
    ]
    xacro_lines.extend(xacro_property(name, value) for name, value in properties)
    xacro_lines.append("</robot>\n")
    xacro_data = "".join(xacro_lines).encode("utf-8")

    controllers = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": update_rate,
                "use_sim_time": True,
                "enforce_command_limits": True,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                "ackermann_steering_controller": {
                    "type": "ackermann_steering_controller/AckermannSteeringController"
                },
            }
        },
        "ackermann_steering_controller": {
            "ros__parameters": {
                "use_sim_time": True,
                "reference_timeout": command_timeout,
                "traction_joints_names": [
                    "rear_right_wheel_joint",
                    "rear_left_wheel_joint",
                ],
                "steering_joints_names": [
                    "front_right_steering_joint",
                    "front_left_steering_joint",
                ],
                "wheelbase": wheelbase,
                "traction_track_width": traction_track,
                "steering_track_width": steering_track,
                "traction_wheels_radius": rear_radius,
                "base_frame_id": "base_footprint",
                "odom_frame_id": "odom",
                "enable_odom_tf": True,
                "open_loop": False,
                "position_feedback": False,
                "reduce_wheel_speed_until_steering_reached": True,
                "velocity_rolling_window_size": 10,
                "pose_covariance_diagonal": [
                    0.01,
                    0.01,
                    1000000.0,
                    1000000.0,
                    1000000.0,
                    0.05,
                ],
                "twist_covariance_diagonal": [
                    0.01,
                    0.01,
                    1000000.0,
                    1000000.0,
                    1000000.0,
                    0.05,
                ],
            }
        },
    }
    header = [
        f"GENERATED by {GENERATOR_ID}; do not edit.",
        f"canonical_geometry_sha256: {geometry_hash}",
        f"mesh_manifest_sha256: {manifest_hash}",
        "status: provisional_assumed; production_ready: false",
    ]
    controllers_data = yaml_bytes(header, controllers)

    guard = {
        "ackermann_command_guard": {
            "ros__parameters": {
                "use_sim_time": True,
                "input_topic": "/cmd_vel_raw",
                "output_topic": "/cmd_vel",
                "base_frame_id": "base_footprint",
                "publish_rate": float(update_rate),
                "command_timeout": command_timeout,
                "max_input_age": command_timeout,
                "future_tolerance": 0.05,
                "zero_linear_epsilon": zero_epsilon,
                "max_projection_iterations": iterations,
                "rolling_wheelbase": wheelbase,
                "steering_track_width": steering_track,
                "traction_track_width": traction_track,
                "traction_wheels_radius": rear_radius,
                "r_min_left": radius_left,
                "r_min_right": radius_right,
                "rear_left_wheel_velocity": rear_left["velocity"],
                "rear_right_wheel_velocity": rear_right["velocity"],
                "rear_left_wheel_acceleration": rear_left["acceleration"],
                "rear_right_wheel_acceleration": rear_right["acceleration"],
                "front_left_steering_lower": steering_left["lower"],
                "front_left_steering_upper": steering_left["upper"],
                "front_left_steering_velocity": steering_left["velocity"],
                "front_right_steering_lower": steering_right["lower"],
                "front_right_steering_upper": steering_right["upper"],
                "front_right_steering_velocity": steering_right["velocity"],
            }
        }
    }
    guard_data = yaml_bytes(header, guard)

    return {
        package_root / "urdf" / "vehicle_geometry.generated.xacro": xacro_data,
        package_root / "config" / "controllers.yaml": controllers_data,
        package_root / "config" / "command_guard.yaml": guard_data,
    }


def check_documents(documents: dict[Path, bytes]) -> None:
    for path, expected in documents.items():
        try:
            actual = path.read_bytes()
        except OSError as exc:
            raise PipelineError(f"cannot read generated file {path}: {exc}") from exc
        if actual != expected:
            raise PipelineError(
                f"{path} is stale: expected sha256={sha256_bytes(expected)}, "
                f"actual sha256={sha256_bytes(actual)}"
            )


def write_documents(documents: dict[Path, bytes]) -> None:
    for path, data in documents.items():
        atomic_write(path, data)


def default_paths() -> tuple[Path, Path, Path]:
    package_root = Path(__file__).resolve().parents[1]
    return (
        package_root,
        package_root / "config" / "vehicle_geometry.yaml",
        package_root / "meshes" / "mesh_manifest.json",
    )


def parse_arguments() -> argparse.Namespace:
    default_root, default_geometry, default_manifest = default_paths()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", type=Path, default=default_root)
    parser.add_argument("--geometry", type=Path, default=default_geometry)
    parser.add_argument("--mesh-manifest", type=Path, default=default_manifest)
    parser.add_argument(
        "--check",
        action="store_true",
        help="recompute and verify all outputs without writing",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        documents = build_documents(
            arguments.geometry.resolve(),
            arguments.mesh_manifest.resolve(),
            arguments.package_root.resolve(),
        )
        if arguments.check:
            check_documents(documents)
            action = "verified"
        else:
            write_documents(documents)
            action = "generated"
        print(f"{action} {len(documents)} geometry-derived files")
        return 0
    except (PipelineError, OSError, ValueError, KeyError) as exc:
        print(f"generate_geometry.py: error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
