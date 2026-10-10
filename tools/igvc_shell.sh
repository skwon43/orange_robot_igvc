#!/usr/bin/env bash

# Host-side helper functions for the Orange Robot IGVC Docker simulation.
#
# Source this file from the repository root (or from any directory):
#
#   source tools/igvc_shell.sh
#
# Then use:
#
#   igvc-sim   # interactive simulation shell
#   igvc-demo  # launch the complete sensor demo
#   igvc-gui   # interactive simulation shell with temporary X11 access

_IGVC_REPO_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd
)"

# Orange Robot IGVC simulation shell.
igvc-sim() (
    cd "${_IGVC_REPO_ROOT}" || exit 1

    docker compose -f compose.sim.yaml run --rm sim bash -lc '
        source /opt/ros/humble/setup.bash

        if [ -f /workspace/install/setup.bash ]; then
            source /workspace/install/setup.bash
        fi

        exec bash
    '
)

# Launch the complete sensor demo from the simulation container.
igvc-demo() (
    cd "${_IGVC_REPO_ROOT}" || exit 1

    docker compose -f compose.sim.yaml run --rm sim bash -lc '
        source /opt/ros/humble/setup.bash

        if [ -f /workspace/install/setup.bash ]; then
            source /workspace/install/setup.bash
        fi

        exec ros2 launch orange_gazebo orange_sensor_demo.launch.xml
    '
)

# Orange Robot IGVC simulation shell with temporary X11 access.
#
# The current simulation container runs as root, so grant access only to the
# local root user rather than broadly enabling all local Docker X11 clients.
igvc-gui() {
    xhost +si:localuser:root >/dev/null 2>&1

    igvc-sim
    local status=$?

    xhost -si:localuser:root >/dev/null 2>&1

    return "${status}"
}
