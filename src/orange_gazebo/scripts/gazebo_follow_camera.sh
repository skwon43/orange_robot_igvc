#!/usr/bin/env bash
set -u

# Configure the Gazebo Fortress GUI camera to follow the spawned Orange robot.
# CameraTracking in Ignition GUI 6 exposes /gui/follow and
# /gui/follow/offset services. The follow offset is expressed in the target's
# local frame, so a negative X offset produces a chase-camera view that rotates
# with the robot.

TARGET="${1:-orange_robot}"
OFFSET_X="${2:--5.0}"
OFFSET_Y="${3:-0.0}"
OFFSET_Z="${4:-3.0}"

FOLLOW_SERVICE="/gui/follow"
OFFSET_SERVICE="/gui/follow/offset"

service_available() {
  ign service -l 2>/dev/null | grep -qx "$1"
}

echo "[gazebo-follow-camera] Waiting for Gazebo GUI camera services..."
for _ in $(seq 1 100); do
  if service_available "${FOLLOW_SERVICE}" && service_available "${OFFSET_SERVICE}"; then
    break
  fi
  sleep 0.1
done

if ! service_available "${FOLLOW_SERVICE}"; then
  echo "[gazebo-follow-camera] ERROR: ${FOLLOW_SERVICE} did not become available." >&2
  exit 1
fi

if ! service_available "${OFFSET_SERVICE}"; then
  echo "[gazebo-follow-camera] ERROR: ${OFFSET_SERVICE} did not become available." >&2
  exit 1
fi

# The robot is spawned dynamically after Gazebo starts. A follow request sent
# before the render scene contains the robot may be discarded, so repeat the
# idempotent requests briefly while the entity appears.
echo "[gazebo-follow-camera] Setting third-person follow target '${TARGET}' with offset (${OFFSET_X}, ${OFFSET_Y}, ${OFFSET_Z})..."
for _ in $(seq 1 40); do
  ign service \
    -s "${FOLLOW_SERVICE}" \
    --reqtype ignition.msgs.StringMsg \
    --reptype ignition.msgs.Boolean \
    --timeout 500 \
    --req "data: '${TARGET}'" >/dev/null 2>&1 || true

  ign service \
    -s "${OFFSET_SERVICE}" \
    --reqtype ignition.msgs.Vector3d \
    --reptype ignition.msgs.Boolean \
    --timeout 500 \
    --req "x: ${OFFSET_X}, y: ${OFFSET_Y}, z: ${OFFSET_Z}" >/dev/null 2>&1 || true

  sleep 0.2
done

echo "[gazebo-follow-camera] Third-person follow request complete. Press Esc in the Gazebo scene to cancel follow mode."
