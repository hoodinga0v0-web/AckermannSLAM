#!/usr/bin/env bash

_ackermann_workspace="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P
)"

source /opt/ros/jazzy/setup.bash
source "${_ackermann_workspace}/local_ros/opt/ros/jazzy/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ -f "${_ackermann_workspace}/install/setup.bash" ]]; then
  source "${_ackermann_workspace}/install/setup.bash"
fi

unset _ackermann_workspace
