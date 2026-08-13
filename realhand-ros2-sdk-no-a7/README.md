# RealHand ROS2 SDK

The package does not vendor hardware drivers. It imports the Python SDK package `realhand` and exposes its hand and arm classes through ROS2 nodes.

## Structure

```text
realhand-ros2-sdk/
  src/realhand_ros2/
    hand/
      model.py      # Hand model metadata, joint names, class resolution
      adapter.py    # Thin wrapper around realhand.L6/O6/L20/L20lite/L25
      node.py       # ROS2 hand node
    arm/
      model.py      # Arm model metadata
      adapter.py    # Thin wrapper around realhand.arm.A7lite/P7
      node.py       # ROS2 arm node
    gui/
      app.py        # PyQt ROS2 topic GUI
  launch/
    hand.launch.py
    dual_hand.launch.py
    arm.launch.py
    gui.launch.py
```

## Supported Python SDK Models

Hands:

- `L6`
- `O6`
- `L20`
- `L20lite`
- `L25`

Arms:

- `A7lite`
- `P7`

## Install

These steps are for Ubuntu 22.04 with ROS2 Humble or Ubuntu 24.04 with ROS2
Jazzy. Use the same system `python3` that ROS2 uses; avoid conda for the ROS2
workspace unless ROS2 is installed inside that conda environment.

### Automated Install

From this checkout, the install script can check/install dependencies, install
the Python SDK, and optionally build a fresh workspace:

```bash
# If ROS2 is already installed:
./install.sh --workspace ~/realhand_ros2_ws

# If ROS2 is not installed yet:
./install.sh --install-ros --workspace ~/realhand_ros2_ws

# For optional A7 Lite kinetix support:
./install.sh --with-arm --workspace ~/realhand_ros2_ws
```

The automated installer copies this repository into
`<workspace>/src/realhand-ros2-sdk` and will stop if that destination already
exists. For a clean reinstall, remove the old workspace first:

```bash
rm -rf ~/realhand_ros2_ws
./install.sh --install-ros --workspace ~/realhand_ros2_ws
```

To keep the existing workspace, choose a different `--workspace` path instead.

The install script defaults to Humble on Ubuntu 22.04 and Jazzy on Ubuntu
24.04. Override it explicitly when needed, for example:

```bash
ROS_DISTRO=jazzy ./install.sh --install-ros --workspace ~/realhand_ros2_ws
```

For local Python SDK development, use an editable checkout:

```bash
./install.sh --local-sdk /path/to/realbot-python-sdk-no-a7 --workspace ~/realhand_ros2_ws
```

### Clean Install Containers

For an isolated install test, use the container launchers in `scripts/`. These
start a fresh Ubuntu shell with this repository mounted read-only at
`/host/realhand-ros2-sdk`, so the install flow copies from the mounted checkout
into a brand-new workspace inside the container.

Ubuntu 22.04 / ROS2 Humble:

```bash
./scripts/run_humble_clean_install_container.sh
```

Ubuntu 24.04 / ROS2 Jazzy:

```bash
./scripts/run_jazzy_clean_install_container.sh
```

Inside the container, run a clean install into an isolated workspace:

```bash
cd /host/realhand-ros2-sdk
./install.sh --install-ros --workspace "${WORKSPACE_DIR}"
source "${WORKSPACE_DIR}/install/setup.bash"
```

The default container workspace path is `/workspaces/realhand_ros2_ws`. Override
it on the host if needed:

```bash
WORKSPACE_DIR=/tmp/p7-test-ws ./scripts/run_jazzy_clean_install_container.sh
```

To test against a local Python SDK checkout instead of downloading the upstream
SDK from GitHub, mount it into the container and use `--local-sdk`:

```bash
LOCAL_SDK_HOST_PATH=/path/to/realbot-python-sdk-main \
./scripts/run_jazzy_clean_install_container.sh
```

Inside the container:

```bash
cd /host/realhand-ros2-sdk
./install.sh --install-ros --local-sdk "${LOCAL_SDK_CONTAINER_PATH}" --workspace "${WORKSPACE_DIR}"
source "${WORKSPACE_DIR}/install/setup.bash"
```

### Manual Prerequisites

Install ROS2 Humble or Jazzy if it is not already installed:

```bash
# Ubuntu 22.04:
export ROS_DISTRO=humble

# Ubuntu 24.04:
# export ROS_DISTRO=jazzy

# Configure a UTF-8 locale.
locale
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable Ubuntu Universe and add the ROS2 apt repository.
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo apt update
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS2.
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-${ROS_DISTRO}-ros-base

# Source ROS2 in every terminal where you build or run this package.
source /opt/ros/${ROS_DISTRO}/setup.bash
```

### Manual ROS2 Workspace Install

```bash
# Ubuntu 22.04:
export ROS_DISTRO=humble

# Ubuntu 24.04:
# export ROS_DISTRO=jazzy

sudo apt update
sudo apt install -y git python3-pip python3-colcon-common-extensions python3-pyqt5

# Source ROS2 before building the workspace.
source /opt/ros/${ROS_DISTRO}/setup.bash

# On Ubuntu 22.04, upgrade pip before installing the Python SDK.
python3 -m pip install --upgrade pip

# Clean install from the upstream Python SDK repository.
python3 -m pip install git+https://github.com/RealHand-Robotics/realbot-python-sdk.git

# Or, for local Python SDK development, install an editable checkout instead:
# python3 -m pip install -e /path/to/realbot-python-sdk-main

# Clone this repository, or copy your existing checkout into the workspace.
mkdir -p realhand_ros2_ws/src
cd realhand_ros2_ws/src
cp -r /path/to/realhand-ros2-sdk .
cd ..

colcon build --symlink-install
source install/setup.bash

ros2 pkg prefix realhand_ros2
ros2 pkg executables realhand_ros2
```

Ubuntu 24.04 marks the system Python environment as externally managed. For a
direct system install, add `--break-system-packages --ignore-installed` to the
`python3 -m pip install` commands above so the SDK is installed into the same
Python interpreter used by ROS2 without trying to uninstall apt-owned Python
packages. Do not upgrade pip with pip on Ubuntu 24.04; use the apt-provided pip:

```bash
python3 -m pip install --break-system-packages --ignore-installed git+https://github.com/RealHand-Robotics/realbot-python-sdk.git
```

If you copy from a previously built checkout, do not reuse its old `build/`,
`install/`, `log/`, or `__pycache__/` directories. A fresh workspace should
rebuild those locally.

For A7 Lite kinetix support, install the Python SDK with its optional kinetix
dependencies instead. P7 uses the Python SDK's LBot TCP controller interface and
does not require the kinetix extra.

```bash
python3 -m pip install "realhand[kinetix] @ git+https://github.com/RealHand-Robotics/realbot-python-sdk.git"

# Or, for local Python SDK development:
# python3 -m pip install -e "/path/to/realbot-python-sdk-main[kinetix]"
```

On Ubuntu 24.04, add `--break-system-packages --ignore-installed` to these
`python3 -m pip install` commands for a direct system install.

## Hand Node

Launch one hand:

```bash
ros2 launch realhand_ros2 hand.launch.py model:=L6 side:=left interface_name:=can0
```

Launch two hands of the same model:

```bash
ros2 launch realhand_ros2 dual_hand.launch.py model:=L6 left_interface:=can0 right_interface:=can1
```

Topics for `side:=left`:

- Subscribe: `/realhand/left/hand/command` (`sensor_msgs/msg/JointState`)
- Subscribe: `/realhand/left/hand/command_json` (`std_msgs/msg/String`)
- Publish: `/realhand/left/hand/state` (`sensor_msgs/msg/JointState`)
- Publish: `/realhand/left/hand/snapshot` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/hand/device_info` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/hand/control_status` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/hand/blocking_result` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/hand/angle` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/hand/speed` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/hand/torque` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/hand/acceleration` (`std_msgs/msg/String`, JSON, models that expose it)
- Publish: `/realhand/left/hand/temperature` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/hand/current` (`std_msgs/msg/String`, JSON, models that expose it)
- Publish: `/realhand/left/hand/touch` (`std_msgs/msg/String`, JSON, alias of force sensor data)
- Publish: `/realhand/left/hand/force_sensor` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/hand/fault` (`std_msgs/msg/String`, JSON, models that expose it)

JointState command fields:

- `position`: target angles, in Python SDK order, range `0..100`
- `velocity`: target speeds, range `0..100`
- `effort`: target torques, range `0..100`

When a command includes speed, torque, and angle targets, the backend applies
speed and torque before sending the angle target.

The hand node starts SDK polling and SDK event streaming by default. Use launch
parameters to disable or tune them:

```bash
ros2 launch realhand_ros2 hand.launch.py model:=L6 side:=left interface_name:=can0 \
  poll_on_start:=true stream_on_start:=true stream_queue_size:=300 \
  poll_intervals_json:='{"angle": 0.03333333333333333, "force_sensor": 0.06666666666666667, "torque": 0.2, "speed": 0.5, "acceleration": 0.5, "temperature": 1.0, "current": 0.5, "fault": 1.0}'
```

By default, the ROS2 backend uses the same sensor/control polling rates as the
Python SDK GUI: angle at 30 Hz, force sensor at 15 Hz, torque at 5 Hz, speed at
2 Hz, acceleration at 2 Hz, temperature at 1 Hz, current at 2 Hz, and fault at
1 Hz. Unsupported sensors are ignored for models that do not expose them.
`poll_intervals_json` values are intervals in seconds, so `0.02` means 50 Hz,
`0.05` means 20 Hz, and `1.0` means 1 Hz.

Use `/realhand/right/...` instead of `/realhand/left/...` when the node is launched
with `side:=right`. Command array lengths must match the selected model:
`L6`/`O6` use 6 values, `L20lite` uses 10 values, and `L20`/`L25` use 16 values.

Hand topic examples:

The examples below use an `L6` left hand. Use `/realhand/right/...` for
`side:=right`, and adjust the command arrays for other hand models.

Most examples use plain `ros2 topic pub --once`. Add
`--wait-matching-subscriptions 1` when you want the one-shot publisher to wait
until it discovers at least one backend subscriber before publishing. This is
safer right after starting a backend because ROS2 discovery can take a moment.

The same angle command without and with subscriber waiting:

```bash
ros2 topic pub --once /realhand/left/hand/command sensor_msgs/msg/JointState \
"{name: ['thumb_flex', 'thumb_abd', 'index', 'middle', 'ring', 'pinky'], position: [100, 100, 100, 100, 100, 100]}"

ros2 topic pub --once --wait-matching-subscriptions 1 /realhand/left/hand/command sensor_msgs/msg/JointState \
"{name: ['thumb_flex', 'thumb_abd', 'index', 'middle', 'ring', 'pinky'], position: [100, 100, 100, 100, 100, 100]}"
```

Send angle targets only:

```bash
ros2 topic pub --once /realhand/left/hand/command sensor_msgs/msg/JointState \
"{name: ['thumb_flex', 'thumb_abd', 'index', 'middle', 'ring', 'pinky'], position: [100, 100, 100, 100, 100, 100]}"
```

Send speed targets only:

```bash
ros2 topic pub --once /realhand/left/hand/command sensor_msgs/msg/JointState \
"{name: ['thumb_flex', 'thumb_abd', 'index', 'middle', 'ring', 'pinky'], velocity: [40, 40, 40, 40, 40, 40]}"
```

Send torque targets only:

```bash
ros2 topic pub --once /realhand/left/hand/command sensor_msgs/msg/JointState \
"{name: ['thumb_flex', 'thumb_abd', 'index', 'middle', 'ring', 'pinky'], effort: [30, 30, 30, 30, 30, 30]}"
```

Send angle and speed:

```bash
ros2 topic pub --once /realhand/left/hand/command sensor_msgs/msg/JointState \
"{name: ['thumb_flex', 'thumb_abd', 'index', 'middle', 'ring', 'pinky'], position: [0, 0, 0, 0, 0, 0], velocity: [80, 80, 80, 80, 80, 80]}"
```

Send angle and torque:

```bash
ros2 topic pub --once /realhand/left/hand/command sensor_msgs/msg/JointState \
"{name: ['thumb_flex', 'thumb_abd', 'index', 'middle', 'ring', 'pinky'], position: [100, 100, 100, 100, 100, 100], effort: [40, 40, 40, 40, 40, 40]}"
```

Send speed and torque without moving to a new angle:

```bash
ros2 topic pub --once /realhand/left/hand/command sensor_msgs/msg/JointState \
"{name: ['thumb_flex', 'thumb_abd', 'index', 'middle', 'ring', 'pinky'], velocity: [60, 60, 60, 60, 60, 60], effort: [50, 50, 50, 50, 50, 50]}"
```

Send angle, speed, and torque in one command:

```bash
ros2 topic pub --once /realhand/left/hand/command sensor_msgs/msg/JointState \
"{name: ['thumb_flex', 'thumb_abd', 'index', 'middle', 'ring', 'pinky'], position: [50, 50, 50, 50, 50, 50], velocity: [60, 60, 60, 60, 60, 60], effort: [30, 30, 30, 30, 30, 30]}"
```

Publish angles, speeds, and torques to `/realhand/left/hand/command_json`:

```bash
ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"angles\": [50, 50, 50, 50, 50, 50], \"speeds\": [60, 60, 60, 60, 60, 60], \"torques\": [30, 30, 30, 30, 30, 30]}'}"
```

Clear hand faults through `/realhand/left/hand/command_json`:

```bash
ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"clear_faults\"}'}"
```

Control SDK polling and SDK stream forwarding through `/realhand/left/hand/command_json`:

Use `start_polling` without `intervals` to restore the launch/default rates, or
include `intervals` to change the rates while the backend is already running.

```bash
ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"start_polling\"}'}"

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"start_polling\", \"intervals\": {\"angle\": 0.03333333333333333, \"force_sensor\": 0.06666666666666667, \"torque\": 0.2, \"speed\": 0.5, \"acceleration\": 0.5, \"temperature\": 1.0, \"current\": 0.5, \"fault\": 1.0}}'}"

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"stop_polling\"}'}"

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"start_stream\", \"maxsize\": 300}'}"

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"stop_stream\"}'}"
```

Read cached SDK snapshots or request SDK `get_blocking()` through `/realhand/left/hand/command_json`.
Results are published on `/realhand/left/hand/blocking_result`; control
acknowledgements are published on `/realhand/left/hand/control_status`.
`get_blocking` runs in a worker thread and pauses polling while it waits by
default, then restores the previous polling intervals.

```bash
ros2 topic echo /realhand/left/hand/control_status
ros2 topic echo /realhand/left/hand/blocking_result

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"get_snapshot\", \"sensor\": \"angle\", \"request_id\": \"angle-cache-1\"}'}"

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"get_blocking\", \"sensor\": \"fault\", \"timeout_ms\": 500, \"request_id\": \"fault-1\"}'}"

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"get_blocking\", \"sensor\": \"force_sensor\", \"timeout_ms\": 1000, \"request_id\": \"force-1\"}'}"
```

Supported `sensor` names depend on the selected hand model. Common names are
`angle`, `speed`, `torque`, `acceleration`, `temperature`, `current`, `fault`,
and `force_sensor`. `touch` is accepted as an alias for `force_sensor`.

Echo the typed hand state from `/realhand/left/hand/state`:

```bash
ros2 topic echo /realhand/left/hand/state
```

Echo the full JSON snapshot from `/realhand/left/hand/snapshot`:

```bash
ros2 topic echo /realhand/left/hand/snapshot
```

Echo device metadata from `/realhand/left/hand/device_info`:

```bash
ros2 topic echo --once --full-length /realhand/left/hand/device_info
```

Read one message from common hand feedback topics:

```bash
ros2 topic echo --once --full-length /realhand/left/hand/state
ros2 topic echo --once --full-length /realhand/left/hand/angle
ros2 topic echo --once --full-length /realhand/left/hand/speed
ros2 topic echo --once --full-length /realhand/left/hand/torque
ros2 topic echo --once --full-length /realhand/left/hand/temperature
ros2 topic echo --once --full-length /realhand/left/hand/current
ros2 topic echo --once --full-length /realhand/left/hand/fault
```

Read force sensor data and touch heatmap matrices. The `touch` topic is an
alias of the force sensor payload and contains per-finger matrices used by the
GUI heatmap:

```bash
ros2 topic echo --once --full-length /realhand/left/hand/force_sensor
ros2 topic echo --once --full-length /realhand/left/hand/touch
```

Subscribe from Python and print hand state, snapshot, device info, control
status, blocking results, touch, temperature, and current:

```bash
python3 examples/subscribe_hand_topics.py --side left
python3 examples/subscribe_hand_topics.py --side left --all-sensors
```

Echo individual hand sensor topics:

```bash
ros2 topic echo /realhand/left/hand/angle
ros2 topic echo /realhand/left/hand/speed
ros2 topic echo /realhand/left/hand/torque
ros2 topic echo /realhand/left/hand/acceleration
ros2 topic echo /realhand/left/hand/temperature
ros2 topic echo /realhand/left/hand/current
ros2 topic echo /realhand/left/hand/touch
ros2 topic echo /realhand/left/hand/force_sensor
ros2 topic echo /realhand/left/hand/fault
```

Check topic types and active publishers/subscribers:

```bash
ros2 topic info -v /realhand/left/hand/command
ros2 topic info -v /realhand/left/hand/command_json
ros2 topic info -v /realhand/left/hand/state
ros2 topic info -v /realhand/left/hand/snapshot
ros2 topic info -v /realhand/left/hand/device_info
ros2 topic info -v /realhand/left/hand/control_status
ros2 topic info -v /realhand/left/hand/blocking_result
ros2 topic info -v /realhand/left/hand/angle
ros2 topic info -v /realhand/left/hand/speed
ros2 topic info -v /realhand/left/hand/torque
ros2 topic info -v /realhand/left/hand/acceleration
ros2 topic info -v /realhand/left/hand/temperature
ros2 topic info -v /realhand/left/hand/current
ros2 topic info -v /realhand/left/hand/touch
ros2 topic info -v /realhand/left/hand/force_sensor
ros2 topic info -v /realhand/left/hand/fault
```

Stop manual subscribers and refresh stale graph entries:

```bash
# ros2 topic echo and the Python subscription examples unsubscribe when the
# process exits. Press Ctrl-C in that terminal to stop them.

# One-shot commands such as "ros2 topic echo --once" and
# "ros2 topic pub --once" exit on their own.

# The backend remains subscribed to command topics while it is running. That is
# expected; stop the backend with Ctrl-C in the launch terminal when finished.

# Check whether backend nodes are still running:
ros2 node list
ps -ef | grep -E 'realhand_hand_node|hand.launch|realhand_arm_node|arm.launch' | grep -v grep

# If a ROS2 graph entry looks stale after a crash, restart the local ROS2 daemon:
ros2 daemon stop
ros2 daemon start

# Only if you intentionally want to stop all matching manual backend processes:
pkill -f realhand_hand_node
pkill -f 'hand.launch.py'
pkill -f realhand_arm_node
pkill -f 'arm.launch.py'
```

JSON hand command payloads accepted by `/realhand/left/hand/command_json`:

```json
{"angles": [50, 50, 50, 50, 50, 50], "speeds": [60, 60, 60, 60, 60, 60]}
{"torques": [30, 30, 30, 30, 30, 30]}
{"action": "clear_faults"}
{"action": "start_polling", "intervals": {"angle": 0.03333333333333333, "force_sensor": 0.06666666666666667, "torque": 0.2, "speed": 0.5, "acceleration": 0.5, "temperature": 1.0, "current": 0.5, "fault": 1.0}}
{"action": "stop_polling"}
{"action": "start_stream", "maxsize": 300}
{"action": "stop_stream"}
{"action": "get_snapshot", "sensor": "angle", "request_id": "angle-cache-1"}
{"action": "get_blocking", "sensor": "fault", "timeout_ms": 500, "request_id": "fault-1"}
{"action": "get_blocking", "sensor": "force_sensor", "timeout_ms": 1000, "request_id": "force-1"}
```

## Arm Node

Launch an A7 Lite SocketCAN arm:

```bash
ros2 launch realhand_ros2 arm.launch.py model:=A7lite side:=left interface_name:=can0 world_frame:=urdf
```

Launch a P7 LBot TCP arm:

```bash
ros2 launch realhand_ros2 arm.launch.py model:=P7 side:=left interface_type:=lbot interface_name:=192.168.10.21 world_frame:=urdf
```

Topics for `side:=left`:

- Subscribe: `/realhand/left/arm/joint_command` (`sensor_msgs/msg/JointState`)
- Subscribe: `/realhand/left/arm/command_json` (`std_msgs/msg/String`)
- Publish: `/realhand/left/arm/joint_state` (`sensor_msgs/msg/JointState`)
- Publish: `/realhand/left/arm/pose` (`std_msgs/msg/String`, JSON)
- Publish: `/realhand/left/arm/control_angle` (`std_msgs/msg/String`, JSON array, when supported)
- Publish: `/realhand/left/arm/control_velocity` (`std_msgs/msg/String`, JSON array, when supported)
- Publish: `/realhand/left/arm/control_acceleration` (`std_msgs/msg/String`, JSON array, when supported)
- Publish: `/realhand/left/arm/temperature` (`std_msgs/msg/String`, JSON array, when supported)
- Publish: `/realhand/left/arm/moving` (`std_msgs/msg/String`, JSON object, when supported)
- Publish: `/realhand/left/arm/joint_limits` (`std_msgs/msg/String`, JSON array, when supported)
- Publish: `/realhand/left/arm/state_snapshot` (`std_msgs/msg/String`, JSON object)

JointState command fields:

- `position`: target joint angles in radians, sent with `move_j(..., blocking=False)`
- `velocity`: joint velocity limits/targets, passed to the Python SDK
- `effort`: joint accelerations, passed to the Python SDK

P7 validates velocity and acceleration values in the range `0..20`.
When one command includes velocity, acceleration, and joint targets, the backend
applies velocity and acceleration before sending the joint target.

Use `/realhand/right/...` instead of `/realhand/left/...` when the node is launched
with `side:=right`. For A7 Lite, `interface_name` is the CAN interface such
as `can0` and `interface_type` is normally `socketcan`. For P7,
`interface_name` is the LBot controller TCP host and `interface_type` must be
`lbot`. If `interface_name` and `interface_type` are omitted, the arm node uses
model-specific defaults: `can0`/`socketcan` for A7 Lite and
`192.168.10.21`/`lbot` for P7.

Arm topic examples:

The examples below use a `left` arm. Use `/realhand/right/...` for
`side:=right`.

Most examples use plain `ros2 topic pub --once`. Add
`--wait-matching-subscriptions 1` when you want the one-shot publisher to wait
until it discovers at least one backend subscriber before publishing. This is
safer right after starting a backend because ROS2 discovery can take a moment.

The same enable command without and with subscriber waiting:

```bash
ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"enable\"}'}"

ros2 topic pub --once --wait-matching-subscriptions 1 /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"enable\"}'}"
```

Enable, disable, reset errors, and emergency stop control:

```bash
ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"enable\"}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"disable\"}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"reset_error\"}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"emergency_stop\", \"enable\": false}'}"
```

Set arm velocity limits/targets:

```bash
ros2 topic pub --once /realhand/left/arm/joint_command sensor_msgs/msg/JointState \
"{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'], velocity: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"velocities\": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}'}"
```

Set arm accelerations:

```bash
ros2 topic pub --once /realhand/left/arm/joint_command sensor_msgs/msg/JointState \
"{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'], effort: [1, 1, 1, 1, 1, 1, 1]}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"accelerations\": [1, 1, 1, 1, 1, 1, 1]}'}"
```

Move joints with `move_j`:

```bash
ros2 topic pub --once /realhand/left/arm/joint_command sensor_msgs/msg/JointState \
"{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'], position: [0, 0, 0, 0, 0, 0, 0]}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"joints\": [0, 0, 0, 0, 0, 0, 0], \"blocking\": false}'}"
```

Move joints with velocity and acceleration in one command. The backend applies
velocity and acceleration before sending the joint target:

```bash
ros2 topic pub --once /realhand/left/arm/joint_command sensor_msgs/msg/JointState \
"{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'], position: [0, 0, 0, 0, 0, 0, 0], velocity: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], effort: [1, 1, 1, 1, 1, 1, 1]}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"velocities\": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], \"accelerations\": [1, 1, 1, 1, 1, 1, 1], \"joints\": [0, 0, 0, 0, 0, 0, 0], \"blocking\": false}'}"
```

Home and emergency stop:

```bash
ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"home\", \"blocking\": false}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"emergency_stop\"}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"emergency_stop\", \"enable\": false}'}"
```

Read one arm state and pose sample:

```bash
ros2 topic echo --once --full-length /realhand/left/arm/joint_state
ros2 topic echo --once --full-length /realhand/left/arm/pose
ros2 topic echo --once --full-length /realhand/left/arm/control_angle
ros2 topic echo --once --full-length /realhand/left/arm/control_velocity
ros2 topic echo --once --full-length /realhand/left/arm/control_acceleration
ros2 topic echo --once --full-length /realhand/left/arm/temperature
ros2 topic echo --once --full-length /realhand/left/arm/moving
ros2 topic echo --once --full-length /realhand/left/arm/joint_limits
ros2 topic echo --once --full-length /realhand/left/arm/state_snapshot
```

Continuously echo the arm state and Cartesian pose:

```bash
ros2 topic echo /realhand/left/arm/joint_state
ros2 topic echo /realhand/left/arm/pose
```

Check topic types and active publishers/subscribers:

```bash
ros2 topic info -v /realhand/left/arm/joint_command
ros2 topic info -v /realhand/left/arm/command_json
ros2 topic info -v /realhand/left/arm/joint_state
ros2 topic info -v /realhand/left/arm/pose
```

JSON arm command payloads accepted by `/realhand/left/arm/command_json`:

```json
{"action": "enable"}
{"action": "disable"}
{"action": "home", "blocking": false}
{"action": "reset_error"}
{"action": "emergency_stop"}
{"action": "emergency_stop", "enable": true}
{"action": "emergency_stop", "enable": false}
{"action": "resume_from_emergency_stop"}
{"joints": [0, 0, 0, 0, 0, 0, 0], "blocking": false}
{"velocities": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}
{"accelerations": [1, 1, 1, 1, 1, 1, 1]}
```

## GUI

The SDK includes a PyQt GUI that follows the Python SDK's hand preset style and talks to the ROS2 bridge topics.

You can either start the hand bridge node first, or open the GUI and press
`Start Backend` after selecting the model, side, and interface:

```bash
ros2 launch realhand_ros2 hand.launch.py model:=L6 side:=left interface_name:=can0
ros2 launch realhand_ros2 gui.launch.py
```

If you do not want to source ROS2 in every terminal, use the included launcher:

```bash
./run_gui.sh
```

The GUI can:

- send hand position, speed, and torque commands
- launch and stop the matching hand backend node from the hand tab
- launch and stop the matching A7 Lite/P7 arm backend node from the arm tab
- autodetect RealHand devices on `can0` through `can3` and fill model, side, and interface
- apply hand presets loaded from the Python SDK when available
- display hand `JointState`, snapshot JSON, device metadata, touch, temperature, and current data
- control hand sensor read mode: stream, snapshot, or `get_blocking`
- start/stop SDK polling and SDK event streaming through ROS2 JSON commands
- request one snapshot or blocking sensor read and display the result
- keep the touch heatmap active in `get_blocking` mode with periodic `force_sensor` reads
- adjust global and per-joint speed/torque from the generic Joint Settings tab
- run touch auto-grab, display commanded torque, and boost finger torque on slip
- run a fault `get_blocking` check from the Hand Test tab and save a CSV report
- send A7 Lite/P7 joint, velocity, acceleration, enable, disable, home, and emergency-stop commands
- display arm joint state and pose JSON

The GUI is a ROS2 client. It does not open the hardware device directly; the
hand and arm `Start Backend` buttons start the matching ROS2 backend nodes for
you.

## Example Scripts

Runnable examples are available in `examples/`:

```bash
./examples/start_backend.sh hand L6 left can0
./examples/read_topics.sh hand left echo-once
python3 examples/subscribe_hand_topics.py --side left
python3 examples/send_hand_command.py --model L6 --side left --positions 50,50,50,50,50,50
python3 examples/arm_read_state.py --side left
python3 examples/arm_move_j.py --model P7 --side left --joints 0,0,0,0,0,0,0
python3 examples/arm_enable_disable.py --side left enable
```

See `examples/README.md` for hand and arm backend, subscription, topic echo,
state-reading, command publishing, enable/disable, motion-setting, `move_j`,
and emergency-stop examples.

## Notes

- This package currently exposes topics only. There are no ROS2 services or actions.
- A7 Lite Cartesian arm features require the Python SDK `kinetix` extra. P7 uses the Python SDK LBot TCP controller interface; set `interface_type:=lbot` and `interface_name` to the controller IP address.
- The GUI requires PyQt5. On Ubuntu, install `python3-pyqt5` if it is not already available.
