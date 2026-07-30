#!/usr/bin/env python3
"""Normalize the supplied binary STL assembly into ROS link-local meshes.

The canonical geometry YAML is the only source of coordinate transforms,
pivots, file mappings, and tolerances.  Source assets are read-only.  Normal
generation writes deterministic binary STL files and a deterministic manifest;
--check recomputes every byte without modifying the output directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
from typing import Any, Iterable

import yaml


TRIANGLE = struct.Struct("<12fH")
UINT32 = struct.Struct("<I")
MANIFEST_NAME = "mesh_manifest.json"
GENERATOR_ID = "ackermann_car_description/normalize_meshes.py:v1"


class PipelineError(RuntimeError):
    """A deterministic input, geometry, or output validation error."""


class UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader which rejects duplicate mapping keys."""


def _construct_mapping(
    loader: UniqueKeyLoader, node: yaml.nodes.MappingNode, deep: bool = False
) -> dict[Any, Any]:
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise PipelineError(
                f"duplicate YAML key {key!r} at line {key_node.start_mark.line + 1}"
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def load_geometry(path: Path) -> tuple[dict[str, Any], bytes]:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise PipelineError(f"cannot read geometry file {path}: {exc}") from exc
    try:
        loaded = yaml.load(raw.decode("utf-8"), Loader=UniqueKeyLoader)
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise PipelineError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(loaded, dict):
        raise PipelineError("canonical geometry root must be a mapping")
    return loaded, raw


def require_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PipelineError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise PipelineError(f"{label} must be finite")
    return result


def require_vector(value: Any, length: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise PipelineError(f"{label} must be a {length}-element list")
    return [require_number(item, f"{label}[{index}]") for index, item in enumerate(value)]


def require_matrix3(value: Any, label: str) -> list[list[float]]:
    if not isinstance(value, list) or len(value) != 3:
        raise PipelineError(f"{label} must be a 3x3 list")
    matrix = [require_vector(row, 3, f"{label}[{index}]") for index, row in enumerate(value)]
    for row in range(3):
        for other in range(3):
            dot = sum(matrix[row][axis] * matrix[other][axis] for axis in range(3))
            expected = 1.0 if row == other else 0.0
            if abs(dot - expected) > 1.0e-9:
                raise PipelineError(f"{label} is not orthonormal")
    determinant = (
        matrix[0][0]
        * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1]
        * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2]
        * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )
    if abs(determinant - 1.0) > 1.0e-9:
        raise PipelineError(f"{label} must be a proper rotation (determinant +1)")
    return matrix


def mat_vec(matrix: list[list[float]], vector: Iterable[float]) -> tuple[float, float, float]:
    values = tuple(vector)
    return tuple(
        sum(matrix[row][column] * values[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def mat_transpose_vec(
    matrix: list[list[float]], vector: Iterable[float]
) -> tuple[float, float, float]:
    values = tuple(vector)
    return tuple(
        sum(matrix[row][column] * values[row] for row in range(3))
        for column in range(3)
    )  # type: ignore[return-value]


def subtract(left: Iterable[float], right: Iterable[float]) -> tuple[float, float, float]:
    lhs = tuple(left)
    rhs = tuple(right)
    return tuple(lhs[index] - rhs[index] for index in range(3))  # type: ignore[return-value]


def add(left: Iterable[float], right: Iterable[float]) -> tuple[float, float, float]:
    lhs = tuple(left)
    rhs = tuple(right)
    return tuple(lhs[index] + rhs[index] for index in range(3))  # type: ignore[return-value]


def norm(vector: Iterable[float]) -> float:
    return math.sqrt(sum(component * component for component in vector))


def cross(
    first: tuple[float, float, float], second: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def dot(first: Iterable[float], second: Iterable[float]) -> float:
    return sum(a * b for a, b in zip(first, second))


def normalized(vector: tuple[float, float, float], label: str) -> tuple[float, float, float]:
    length = norm(vector)
    if length <= 1.0e-15:
        raise PipelineError(f"zero-length vector while computing {label}")
    return tuple(component / length for component in vector)  # type: ignore[return-value]


def read_binary_stl(path: Path) -> dict[str, Any]:
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise PipelineError(f"cannot read STL {path}: {exc}") from exc
    if len(data) < 84:
        raise PipelineError(f"{path} is too short to be a binary STL")
    triangle_count = UINT32.unpack_from(data, 80)[0]
    expected_size = 84 + triangle_count * TRIANGLE.size
    if len(data) != expected_size:
        raise PipelineError(
            f"{path} size {len(data)} does not match binary STL triangle count "
            f"{triangle_count} ({expected_size} bytes expected)"
        )
    triangles: list[dict[str, Any]] = []
    for index in range(triangle_count):
        unpacked = TRIANGLE.unpack_from(data, 84 + index * TRIANGLE.size)
        values = unpacked[:12]
        if not all(math.isfinite(value) for value in values):
            raise PipelineError(f"{path} triangle {index} contains NaN or infinity")
        triangles.append(
            {
                "normal": tuple(float(value) for value in unpacked[0:3]),
                "vertices": (
                    tuple(float(value) for value in unpacked[3:6]),
                    tuple(float(value) for value in unpacked[6:9]),
                    tuple(float(value) for value in unpacked[9:12]),
                ),
                "attribute": int(unpacked[12]),
            }
        )
    return {
        "bytes": data,
        "header": data[:80],
        "triangle_count": triangle_count,
        "triangles": triangles,
    }


def transform_stl(
    source: dict[str, Any],
    source_name: str,
    target_name: str,
    rotation: list[list[float]],
    pivot: list[float],
) -> tuple[bytes, list[dict[str, Any]]]:
    transformed: list[dict[str, Any]] = []
    output = bytearray()
    label = f"ROS normalized {source_name} -> {target_name}".encode("ascii", "replace")
    output.extend(label[:80].ljust(80, b"\0"))
    output.extend(UINT32.pack(source["triangle_count"]))

    for index, triangle in enumerate(source["triangles"]):
        vertices = tuple(
            mat_vec(rotation, subtract(vertex, pivot)) for vertex in triangle["vertices"]
        )
        edge_one = subtract(vertices[1], vertices[0])
        edge_two = subtract(vertices[2], vertices[0])
        normal = normalized(cross(edge_one, edge_two), f"{source_name} triangle {index}")
        values = (
            *normal,
            *vertices[0],
            *vertices[1],
            *vertices[2],
            triangle["attribute"],
        )
        output.extend(TRIANGLE.pack(*values))
        transformed.append(
            {
                "normal": normal,
                "vertices": vertices,
                "attribute": triangle["attribute"],
            }
        )
    return bytes(output), transformed


def _vertex_key(vertex: Iterable[float]) -> tuple[float, float, float]:
    return tuple(float(component) for component in vertex)  # type: ignore[return-value]


def analyze_triangles(triangles: list[dict[str, Any]]) -> dict[str, Any]:
    if not triangles:
        raise PipelineError("STL must contain at least one triangle")
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    edge_counts: dict[
        tuple[tuple[float, float, float], tuple[float, float, float]], list[int]
    ] = {}
    signed_volume = 0.0
    surface_area = 0.0
    degenerate = 0
    minimum_normal_alignment = 1.0
    unique_vertices: set[tuple[float, float, float]] = set()

    for face_index, triangle in enumerate(triangles):
        vertices = tuple(_vertex_key(vertex) for vertex in triangle["vertices"])
        for vertex in vertices:
            unique_vertices.add(vertex)
            for axis in range(3):
                minimum[axis] = min(minimum[axis], vertex[axis])
                maximum[axis] = max(maximum[axis], vertex[axis])
        edge_one = subtract(vertices[1], vertices[0])
        edge_two = subtract(vertices[2], vertices[0])
        geometric = cross(edge_one, edge_two)
        twice_area = norm(geometric)
        if twice_area <= 1.0e-12:
            degenerate += 1
        else:
            surface_area += 0.5 * twice_area
            stored = normalized(tuple(triangle["normal"]), "stored STL normal")
            alignment = dot(stored, tuple(component / twice_area for component in geometric))
            minimum_normal_alignment = min(minimum_normal_alignment, alignment)
        signed_volume += dot(vertices[0], cross(vertices[1], vertices[2])) / 6.0
        for first, second in (
            (vertices[0], vertices[1]),
            (vertices[1], vertices[2]),
            (vertices[2], vertices[0]),
        ):
            key = tuple(sorted((first, second)))  # type: ignore[assignment]
            edge_counts.setdefault(key, []).append(face_index)

    boundary_edges = sum(1 for faces in edge_counts.values() if len(faces) == 1)
    nonmanifold_edges = sum(1 for faces in edge_counts.values() if len(faces) > 2)

    parent = list(range(len(triangles)))

    def find(item: int) -> int:
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    def union(first: int, second: int) -> None:
        first_root = find(first)
        second_root = find(second)
        if first_root != second_root:
            parent[second_root] = first_root

    for faces in edge_counts.values():
        for face in faces[1:]:
            union(faces[0], face)
    shells = len({find(face) for face in range(len(triangles))})

    return {
        "triangles": len(triangles),
        "unique_vertices": len(unique_vertices),
        "bbox_min_u": minimum,
        "bbox_max_u": maximum,
        "bbox_size_u": [maximum[axis] - minimum[axis] for axis in range(3)],
        "surface_area_u2": surface_area,
        "signed_volume_u3": signed_volume,
        "degenerate_triangles": degenerate,
        "boundary_edges": boundary_edges,
        "nonmanifold_edges": nonmanifold_edges,
        "shells": shells,
        "minimum_normal_alignment": minimum_normal_alignment,
    }


def validate_topology(
    analysis: dict[str, Any], label: str, minimum_alignment: float
) -> None:
    if analysis["degenerate_triangles"] != 0:
        raise PipelineError(f"{label} has degenerate triangles")
    if analysis["boundary_edges"] != 0:
        raise PipelineError(f"{label} is not watertight")
    if analysis["nonmanifold_edges"] != 0:
        raise PipelineError(f"{label} has non-manifold edges")
    if analysis["shells"] != 1:
        raise PipelineError(f"{label} must contain exactly one connected shell")
    if analysis["minimum_normal_alignment"] < minimum_alignment:
        raise PipelineError(
            f"{label} normal alignment {analysis['minimum_normal_alignment']:.9g} "
            f"is below {minimum_alignment:.9g}"
        )
    if analysis["signed_volume_u3"] <= 0.0:
        raise PipelineError(f"{label} must have positive signed volume")


def validate_geometry_schema(geometry: dict[str, Any]) -> dict[str, Any]:
    if geometry.get("schema_version") != 1:
        raise PipelineError("vehicle_geometry.yaml schema_version must be 1")
    if geometry.get("model") != "controller_compatible_A":
        raise PipelineError("mesh pipeline only supports model controller_compatible_A")
    status = geometry.get("status")
    if not isinstance(status, dict) or status.get("classification") != "provisional_assumed":
        raise PipelineError("fallback geometry must be explicitly provisional_assumed")
    coordinate = geometry.get("coordinate_system")
    if not isinstance(coordinate, dict):
        raise PipelineError("coordinate_system must be a mapping")
    rotation = require_matrix3(coordinate.get("R_cad_to_ros"), "R_cad_to_ros")
    scale = require_number(coordinate.get("mesh_export_unit"), "mesh_export_unit")
    if scale <= 0.0:
        raise PipelineError("mesh_export_unit must be positive")
    base_pivot = require_vector(
        coordinate.get("base_pivot_cad_u"), 3, "base_pivot_cad_u"
    )
    meshes = geometry.get("meshes")
    if not isinstance(meshes, list):
        raise PipelineError("meshes must be a list")
    validation = geometry.get("validation")
    if not isinstance(validation, dict):
        raise PipelineError("validation must be a mapping")
    expected_inputs = int(
        require_number(validation.get("expected_input_meshes"), "expected_input_meshes")
    )
    expected_outputs = int(
        require_number(validation.get("expected_output_meshes"), "expected_output_meshes")
    )
    if len(meshes) != expected_inputs:
        raise PipelineError(
            f"geometry declares {len(meshes)} meshes, expected {expected_inputs}"
        )
    included = [entry for entry in meshes if isinstance(entry, dict) and entry.get("include")]
    if len(included) != expected_outputs:
        raise PipelineError(
            f"geometry includes {len(included)} meshes, expected {expected_outputs}"
        )
    source_names: set[str] = set()
    target_names: set[str] = set()
    for index, entry in enumerate(meshes):
        if not isinstance(entry, dict):
            raise PipelineError(f"meshes[{index}] must be a mapping")
        source = entry.get("source")
        if not isinstance(source, str) or not source.endswith(".stl") or Path(source).name != source:
            raise PipelineError(f"meshes[{index}].source must be a plain .stl filename")
        if source in source_names:
            raise PipelineError(f"duplicate source mesh {source}")
        source_names.add(source)
        require_vector(entry.get("pivot_cad_u"), 3, f"meshes[{index}].pivot_cad_u")
        expected_triangles = int(
            require_number(
                entry.get("expected_triangles"), f"meshes[{index}].expected_triangles"
            )
        )
        if expected_triangles <= 0:
            raise PipelineError(f"{source} expected_triangles must be positive")
        if entry.get("include"):
            target = entry.get("target")
            if (
                not isinstance(target, str)
                or not target.endswith(".stl")
                or Path(target).name != target
            ):
                raise PipelineError(f"{source} target must be a plain .stl filename")
            if target in target_names:
                raise PipelineError(f"duplicate target mesh {target}")
            target_names.add(target)
            require_vector(
                entry.get("model_origin_ros_m"),
                3,
                f"meshes[{index}].model_origin_ros_m",
            )
        elif entry.get("target") is not None:
            raise PipelineError(f"excluded mesh {source} must have a null target")
    excluded = [entry["source"] for entry in meshes if not entry.get("include")]
    if excluded != ["KnuckleLink.stl"]:
        raise PipelineError("KnuckleLink.stl must be the sole excluded source mesh")
    return {
        "rotation": rotation,
        "scale": scale,
        "base_pivot": base_pivot,
        "meshes": meshes,
        "validation": validation,
    }


def json_ready_analysis(analysis: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in analysis.items():
        if isinstance(value, float):
            result[key] = float(f"{value:.12g}")
        elif isinstance(value, list):
            result[key] = [
                float(f"{item:.12g}") if isinstance(item, float) else item for item in value
            ]
        else:
            result[key] = value
    return result


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o644)
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except BaseException:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass
        raise


def build_outputs(
    input_directory: Path,
    output_directory: Path,
    geometry_path: Path,
) -> tuple[dict[str, bytes], bytes]:
    geometry, geometry_raw = load_geometry(geometry_path)
    schema = validate_geometry_schema(geometry)
    rotation = schema["rotation"]
    scale = schema["scale"]
    base_pivot = schema["base_pivot"]
    validation = schema["validation"]
    reconstruction_tolerance = require_number(
        validation.get("reconstruction_tolerance_u"), "reconstruction_tolerance_u"
    )
    assembly_tolerance = require_number(
        validation.get("assembly_tolerance_m"), "assembly_tolerance_m"
    )
    minimum_alignment = require_number(
        validation.get("normal_alignment_minimum"), "normal_alignment_minimum"
    )

    configured_sources = {entry["source"] for entry in schema["meshes"]}
    actual_sources = {path.name for path in input_directory.glob("*.stl")}
    if actual_sources != configured_sources:
        missing = sorted(configured_sources - actual_sources)
        unexpected = sorted(actual_sources - configured_sources)
        raise PipelineError(
            f"source STL set mismatch; missing={missing}, unexpected={unexpected}"
        )

    source_records: dict[str, Any] = {}
    output_records: list[dict[str, Any]] = []
    output_bytes: dict[str, bytes] = {}

    for entry in schema["meshes"]:
        source_name = entry["source"]
        source_path = input_directory / source_name
        source = read_binary_stl(source_path)
        if source["triangle_count"] != int(entry["expected_triangles"]):
            raise PipelineError(
                f"{source_name} has {source['triangle_count']} triangles, "
                f"expected {entry['expected_triangles']}"
            )
        source_analysis = analyze_triangles(source["triangles"])
        validate_topology(source_analysis, source_name, minimum_alignment)
        source_records[source_name] = {
            "sha256": sha256_bytes(source["bytes"]),
            "included": bool(entry["include"]),
            "triangles": source["triangle_count"],
            "topology": json_ready_analysis(source_analysis),
        }
        if not entry["include"]:
            continue

        target_name = entry["target"]
        pivot = require_vector(entry["pivot_cad_u"], 3, f"{source_name}.pivot_cad_u")
        target_data, transformed_triangles = transform_stl(
            source, source_name, target_name, rotation, pivot
        )
        target = read_binary_stl_from_bytes(target_data, target_name)
        target_analysis = analyze_triangles(target["triangles"])
        validate_topology(target_analysis, target_name, minimum_alignment)

        maximum_reconstruction_error = 0.0
        for source_triangle, target_triangle in zip(
            source["triangles"], target["triangles"]
        ):
            for source_vertex, target_vertex in zip(
                source_triangle["vertices"], target_triangle["vertices"]
            ):
                reconstructed = add(
                    mat_transpose_vec(rotation, target_vertex),
                    pivot,
                )
                maximum_reconstruction_error = max(
                    maximum_reconstruction_error,
                    norm(subtract(reconstructed, source_vertex)),
                )
        if maximum_reconstruction_error > reconstruction_tolerance:
            raise PipelineError(
                f"{target_name} reconstruction error {maximum_reconstruction_error:.9g} u "
                f"exceeds {reconstruction_tolerance:.9g} u"
            )

        inferred_origin_u = mat_vec(rotation, subtract(pivot, base_pivot))
        inferred_origin_m = [component * scale for component in inferred_origin_u]
        model_origin_m = require_vector(
            entry["model_origin_ros_m"], 3, f"{source_name}.model_origin_ros_m"
        )
        origin_delta_m = norm(subtract(model_origin_m, inferred_origin_m))
        if origin_delta_m > assembly_tolerance:
            raise PipelineError(
                f"{target_name} model-origin correction {origin_delta_m:.9g} m "
                f"exceeds {assembly_tolerance:.9g} m"
            )

        output_bytes[target_name] = target_data
        output_records.append(
            {
                "source": source_name,
                "target": target_name,
                "role": entry.get("role", ""),
                "source_sha256": source_records[source_name]["sha256"],
                "target_sha256": sha256_bytes(target_data),
                "pivot_cad_u": pivot,
                "inferred_origin_ros_m": inferred_origin_m,
                "model_origin_ros_m": model_origin_m,
                "model_origin_correction_m": list(
                    subtract(model_origin_m, inferred_origin_m)
                ),
                "model_origin_correction_norm_m": origin_delta_m,
                "maximum_round_trip_error_u": maximum_reconstruction_error,
                "topology": json_ready_analysis(target_analysis),
            }
        )

    output_records.sort(key=lambda record: record["target"])
    expected_targets = int(validation["expected_output_meshes"])
    if len(output_records) != expected_targets or len(output_bytes) != expected_targets:
        raise PipelineError("internal 10-to-9 mapping validation failed")

    manifest = {
        "schema_version": 1,
        "generator": GENERATOR_ID,
        "status": geometry["status"]["classification"],
        "production_ready": bool(geometry["status"].get("production_ready", False)),
        "canonical_geometry": str(geometry_path.resolve()),
        "canonical_geometry_sha256": sha256_bytes(geometry_raw),
        "source_directory": str(input_directory.resolve()),
        "output_directory": str(output_directory.resolve()),
        "mesh_export_unit_m_per_u": scale,
        "R_cad_to_ros": rotation,
        "base_pivot_cad_u": base_pivot,
        "expected_input_count": int(validation["expected_input_meshes"]),
        "expected_output_count": expected_targets,
        "source_meshes": {
            key: source_records[key] for key in sorted(source_records)
        },
        "output_meshes": output_records,
    }
    manifest_data = (
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    return output_bytes, manifest_data


def read_binary_stl_from_bytes(data: bytes, label: str) -> dict[str, Any]:
    if len(data) < 84:
        raise PipelineError(f"generated {label} is too short")
    triangle_count = UINT32.unpack_from(data, 80)[0]
    expected_size = 84 + triangle_count * TRIANGLE.size
    if len(data) != expected_size:
        raise PipelineError(f"generated {label} has an invalid byte count")
    triangles: list[dict[str, Any]] = []
    for index in range(triangle_count):
        unpacked = TRIANGLE.unpack_from(data, 84 + index * TRIANGLE.size)
        triangles.append(
            {
                "normal": tuple(float(value) for value in unpacked[0:3]),
                "vertices": (
                    tuple(float(value) for value in unpacked[3:6]),
                    tuple(float(value) for value in unpacked[6:9]),
                    tuple(float(value) for value in unpacked[9:12]),
                ),
                "attribute": int(unpacked[12]),
            }
        )
    return {
        "bytes": data,
        "header": data[:80],
        "triangle_count": triangle_count,
        "triangles": triangles,
    }


def check_outputs(
    output_directory: Path, expected_meshes: dict[str, bytes], manifest_data: bytes
) -> None:
    actual_meshes = {path.name for path in output_directory.glob("*.stl")}
    expected_names = set(expected_meshes)
    if actual_meshes != expected_names:
        raise PipelineError(
            f"normalized STL set is stale; expected={sorted(expected_names)}, "
            f"actual={sorted(actual_meshes)}"
        )
    for name, expected in sorted(expected_meshes.items()):
        path = output_directory / name
        try:
            actual = path.read_bytes()
        except OSError as exc:
            raise PipelineError(f"cannot read generated mesh {path}: {exc}") from exc
        if actual != expected:
            raise PipelineError(
                f"{name} is stale: expected sha256={sha256_bytes(expected)}, "
                f"actual sha256={sha256_bytes(actual)}"
            )
    manifest_path = output_directory / MANIFEST_NAME
    try:
        actual_manifest = manifest_path.read_bytes()
    except OSError as exc:
        raise PipelineError(f"cannot read manifest {manifest_path}: {exc}") from exc
    if actual_manifest != manifest_data:
        raise PipelineError(
            f"{MANIFEST_NAME} is stale: expected sha256={sha256_bytes(manifest_data)}, "
            f"actual sha256={sha256_bytes(actual_manifest)}"
        )


def write_outputs(
    output_directory: Path, expected_meshes: dict[str, bytes], manifest_data: bytes
) -> None:
    output_directory.mkdir(parents=True, exist_ok=True)
    unexpected = {
        path.name for path in output_directory.glob("*.stl")
    } - set(expected_meshes)
    if unexpected:
        raise PipelineError(
            f"refusing to overwrite output directory with unexpected STL files: "
            f"{sorted(unexpected)}"
        )
    for name, data in sorted(expected_meshes.items()):
        atomic_write(output_directory / name, data)
    atomic_write(output_directory / MANIFEST_NAME, manifest_data)


def default_paths() -> tuple[Path, Path, Path]:
    package_root = Path(__file__).resolve().parents[1]
    workspace_root = package_root.parents[2]
    return (
        workspace_root / "slam_files",
        package_root / "config" / "vehicle_geometry.yaml",
        package_root / "meshes",
    )


def parse_arguments() -> argparse.Namespace:
    default_input, default_geometry, default_output = default_paths()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=default_input)
    parser.add_argument("--geometry", type=Path, default=default_geometry)
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument(
        "--check",
        action="store_true",
        help="recompute and verify all outputs without writing",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        expected_meshes, manifest_data = build_outputs(
            arguments.input.resolve(),
            arguments.output.resolve(),
            arguments.geometry.resolve(),
        )
        if arguments.check:
            check_outputs(arguments.output.resolve(), expected_meshes, manifest_data)
            action = "verified"
        else:
            write_outputs(arguments.output.resolve(), expected_meshes, manifest_data)
            action = "generated"
        print(
            f"{action} {len(expected_meshes)} normalized STL files and "
            f"{MANIFEST_NAME}"
        )
        return 0
    except (PipelineError, OSError, ValueError, struct.error) as exc:
        print(f"normalize_meshes.py: error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
