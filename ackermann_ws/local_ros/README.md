# Local ROS 2 Jazzy overlay

This directory contains unmodified files extracted from official ROS 2 Jazzy
Debian packages. It exists to keep this workspace reproducible on the current
host, whose partially updated Fast DDS / Fast-CDR and ros2_control packages are
not mutually ABI-compatible.

`../setup_local.bash` overlays this prefix before the workspace install and
selects `rmw_cyclonedds_cpp`. Do not source this directory by itself, mix it
with Fast DDS at runtime, or edit extracted library files.

## Package provenance

All packages came from `https://packages.ros.org/ros2/ubuntu`, distribution
`noble/main`, architecture `amd64`.

| Debian package | Exact version |
|---|---|
| `ros-jazzy-cyclonedds` | `0.10.5-1noble.20260225.142613` |
| `ros-jazzy-fastcdr` | `2.2.7-1noble.20260225.051855` |
| `ros-jazzy-gz-ros2-control` | `1.2.19-1noble.20260615.171757` |
| `ros-jazzy-iceoryx-binding-c` | `2.0.6-1noble.20260225.140829` |
| `ros-jazzy-iceoryx-hoofs` | `2.0.6-1noble.20260225.055330` |
| `ros-jazzy-iceoryx-posh` | `2.0.6-1noble.20260225.135341` |
| `ros-jazzy-rmw-cyclonedds-cpp` | `2.2.3-1noble.20260615.123728` |
| `ros-jazzy-ros2-control-cmake` | `0.4.0-1noble.20260429.101922` |

Each package retains its Debian changelog and copyright file under
`usr/share/doc/ros-jazzy-*/`. Upstream license files and package metadata
remain under `opt/ros/jazzy/share/` where supplied.

`gz_ros2_control` from this prefix is only a bootstrap copy. The pinned
`src/gz_ros2_control` source is rebuilt against the active system
ros2_control ABI, and the resulting `install/gz_ros2_control` takes runtime
precedence.

`COLCON_IGNORE` prevents extracted content from being mistaken for source
packages. If the workspace is moved, rebuild `install/`; symlink-install
artifacts are intentionally path-specific.
