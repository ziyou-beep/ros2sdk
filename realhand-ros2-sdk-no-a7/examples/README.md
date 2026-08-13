# RealHand ROS2 Example Scripts

These examples assume the package has been built and sourced:

```bash
colcon build --packages-select realhand_ros2
source install/setup.bash
```

Start a hand backend:

```bash
./examples/start_backend.sh hand L6 left can0
```

Start a hand backend with explicit SDK polling and event stream settings:

```bash
ros2 launch realhand_ros2 hand.launch.py model:=L6 side:=left interface_name:=can0 \
  poll_on_start:=true stream_on_start:=true stream_queue_size:=300 \
  poll_intervals_json:='{"angle": 0.03333333333333333, "force_sensor": 0.06666666666666667, "torque": 0.2, "speed": 0.5, "acceleration": 0.5, "temperature": 1.0, "current": 0.5, "fault": 1.0}'
```

Start an arm backend:

```bash
./examples/start_backend.sh arm A7lite left can0
./examples/start_backend.sh arm P7 left 192.168.10.21 lbot
```

List, inspect, or echo topics:

```bash
./examples/read_topics.sh hand left list
./examples/read_topics.sh hand left info
./examples/read_topics.sh hand left echo-once
./examples/read_topics.sh arm left echo
```

Read hand device metadata:

```bash
ros2 topic echo --once --full-length /realhand/left/hand/device_info
```

Echo hand control status and asynchronous `get_blocking`/`get_snapshot` results
while sending runtime commands:

```bash
ros2 topic echo /realhand/left/hand/control_status
ros2 topic echo /realhand/left/hand/blocking_result
```

Read common hand feedback topics:

```bash
ros2 topic echo --once --full-length /realhand/left/hand/state
ros2 topic echo --once --full-length /realhand/left/hand/angle
ros2 topic echo --once --full-length /realhand/left/hand/speed
ros2 topic echo --once --full-length /realhand/left/hand/torque
ros2 topic echo --once --full-length /realhand/left/hand/temperature
ros2 topic echo --once --full-length /realhand/left/hand/current
```

Read force sensor data and touch heatmap matrices:

```bash
ros2 topic echo --once --full-length /realhand/left/hand/force_sensor
ros2 topic echo --once --full-length /realhand/left/hand/touch
```

Subscribe from Python:

```bash
python3 examples/subscribe_hand_topics.py --side left
python3 examples/subscribe_hand_topics.py --side left --all-sensors
python3 examples/subscribe_arm_topics.py --side left
python3 examples/arm_read_state.py --side left
```

Send hand commands:

```bash
python3 examples/send_hand_command.py --model L6 --side left --positions 50,50,50,50,50,50
python3 examples/send_hand_command.py --model L6 --side left --velocities 40,40,40,40,40,40
python3 examples/send_hand_command.py --model L6 --side left --torques 30,30,30,30,30,30
python3 examples/send_hand_command.py --model L6 --side left --positions 0,0,0,0,0,0 --velocities 80,80,80,80,80,80
python3 examples/send_hand_command.py --model L6 --side left --positions 100,100,100,100,100,100 --torques 40,40,40,40,40,40
python3 examples/send_hand_command.py --model L6 --side left --velocities 60,60,60,60,60,60 --torques 50,50,50,50,50,50
python3 examples/send_hand_command.py --model L6 --side left --positions 50,50,50,50,50,50 --velocities 60,60,60,60,60,60 --torques 30,30,30,30,30,30
python3 examples/send_hand_command.py --model L6 --side left --open
python3 examples/send_hand_command.py --model L6 --side left --close
python3 examples/send_hand_command.py --side left --json '{"action": "clear_faults"}'
python3 examples/send_hand_command.py --side left --json '{"action": "start_polling", "intervals": {"angle": 0.03333333333333333, "force_sensor": 0.06666666666666667, "torque": 0.2, "speed": 0.5, "acceleration": 0.5, "temperature": 1.0, "current": 0.5, "fault": 1.0}}'
python3 examples/send_hand_command.py --side left --json '{"action": "stop_polling"}'
python3 examples/send_hand_command.py --side left --json '{"action": "start_stream", "maxsize": 300}'
python3 examples/send_hand_command.py --side left --json '{"action": "stop_stream"}'
python3 examples/send_hand_command.py --side left --json '{"action": "get_snapshot", "sensor": "angle", "request_id": "angle-cache-1"}'
python3 examples/send_hand_command.py --side left --json '{"action": "get_blocking", "sensor": "fault", "timeout_ms": 500, "request_id": "fault-1"}'
```

Send the same polling, stream, snapshot, and blocking commands with raw ROS2
topic publishing:

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

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"get_snapshot\", \"sensor\": \"angle\", \"request_id\": \"angle-cache-1\"}'}"

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"get_blocking\", \"sensor\": \"fault\", \"timeout_ms\": 500, \"request_id\": \"fault-1\"}'}"

ros2 topic pub --once /realhand/left/hand/command_json std_msgs/msg/String \
"{data: '{\"action\": \"get_blocking\", \"sensor\": \"force_sensor\", \"timeout_ms\": 1000, \"request_id\": \"force-1\"}'}"
```

Send arm commands with raw ROS2 topic publishing:

```bash
# Enable, disable, reset errors, and emergency stop control.
ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"enable\"}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"disable\"}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"reset_error\"}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"emergency_stop\", \"enable\": false}'}"

# Use --wait-matching-subscriptions 1 when the backend just started.
ros2 topic pub --once --wait-matching-subscriptions 1 /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"enable\"}'}"

# Set velocities and accelerations.
ros2 topic pub --once /realhand/left/arm/joint_command sensor_msgs/msg/JointState \
"{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'], velocity: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"

ros2 topic pub --once /realhand/left/arm/joint_command sensor_msgs/msg/JointState \
"{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'], effort: [1, 1, 1, 1, 1, 1, 1]}"

# Move joints with move_j.
ros2 topic pub --once /realhand/left/arm/joint_command sensor_msgs/msg/JointState \
"{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'], position: [0, 0, 0, 0, 0, 0, 0]}"

# Move joints with velocity and acceleration applied first.
ros2 topic pub --once /realhand/left/arm/joint_command sensor_msgs/msg/JointState \
"{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'], position: [0, 0, 0, 0, 0, 0, 0], velocity: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], effort: [1, 1, 1, 1, 1, 1, 1]}"

# Home and emergency stop.
ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"home\", \"blocking\": false}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"emergency_stop\"}'}"

ros2 topic pub --once /realhand/left/arm/command_json std_msgs/msg/String \
"{data: '{\"action\": \"emergency_stop\", \"enable\": false}'}"

# Read state and pose once.
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

Send arm commands:

```bash
python3 examples/send_arm_command.py --model A7lite --side left --joints 0,0,0,0,0,0,0
python3 examples/send_arm_command.py --model P7 --side left --joints 0,0,0,0,0,0,0
python3 examples/send_arm_command.py --side left --action enable
python3 examples/send_arm_command.py --side left --action emergency_stop
python3 examples/send_arm_command.py --side left --json '{"action": "emergency_stop", "enable": false}'
python3 examples/send_arm_command.py --side left --action resume_from_emergency_stop
```

Use the focused arm examples for the main SDK-style functions:

```bash
# Enable, disable, reset errors, or resume after emergency stop.
python3 examples/arm_enable_disable.py --side left enable
python3 examples/arm_enable_disable.py --side left disable
python3 examples/arm_enable_disable.py --side left reset_error
python3 examples/arm_enable_disable.py --side left resume_from_emergency_stop

# Set velocity and acceleration values before motion.
python3 examples/arm_set_velocity_acceleration.py --model P7 --side left \
  --velocities 0.5,0.5,0.5,0.5,0.5,0.5,0.5 \
  --accelerations 1,1,1,1,1,1,1

# Move joints with move_j. Velocities and accelerations in the same command
# are applied before the joint target.
python3 examples/arm_move_j.py --model P7 --side left \
  --joints 0,0,0,0,0,0,0 \
  --velocities 0.5,0.5,0.5,0.5,0.5,0.5,0.5 \
  --accelerations 1,1,1,1,1,1,1

# Read one joint_state and pose sample.
python3 examples/arm_read_state.py --side left

# Emergency stop.
python3 examples/arm_emergency_stop.py --side left

# Resume after emergency stop.
python3 examples/send_arm_command.py --side left --action resume_from_emergency_stop

# Move, then emergency stop after a short delay.
bash examples/arm_move_then_emergency_stop.sh left 0.5
```
