# openflex_teleop_exo

OpenFlex EXO whole-body teleoperation adapter. This package converts EXO exoskeleton data into OpenFlex chassis, lift column, head, and dual-arm control commands.

Chinese documentation: [README_zh.md](README_zh.md)

## Feature Scope

- Receives EXO WebSocket outputs on `/exo/joint_command` and `/exo/gamepad_keys`.
- Dual arms: retargets `/exo/joint_command` to left/right arm target joints, then bridges them to OpenFlex dual-arm position controllers.
- Chassis: EXO left joystick controls translation, and right joystick X controls rotation.
- Lift column: EXO left red/blue buttons control lift down/up.
- Head: EXO right red/blue/white/black buttons control pitch and yaw.
- Speed scale: subscribes to and publishes `/openflex_teleop_exo/speed_scale`, which can be adjusted dynamically with the RViz panel.

## Package Structure

- `openflex_exo_teleop_node.py`: chassis, lift column, head, and speed-scale control.
- `arm_retargeting_node.py`: retargets EXO dual-arm joints to OpenFlex dual-arm joints.
- `arm_bridge_node.py`: dual-arm safety checking, smooth handover, and controller command publishing.
- `config/openflex_teleop_exo.yaml`: whole-body control parameters.
- `config/retargeting_OpenArmX.yaml`: dual-arm retargeting parameters.
- `launch/websocket_publisher.launch.py`: starts the EXO WebSocket receiver/publisher node.
- `launch/exo_retargeting.launch.py`: starts the OpenFlex EXO mapping and control chain.
- `launch/openflex_teleop_exo.launch.py`: safe default launch, starts only the mapping chain and does not start WebSocket.
- `launch/openflex_full_exo_teleop.launch.py`: starts WebSocket and the mapping chain together.

## Dependency Notes

`websocket_publisher.launch.py` currently reuses the mature WebSocket receiver from `openarmx_teleop_exo`:

```bash
openarmx_teleop_exo websocket_teleoperator
```

OpenFlex-specific mapping, dual-arm retargeting, bridge, chassis, lift column, and head-control logic are implemented in this package.

Therefore, if `websocket_publisher.launch.py` is still needed, do not delete `~/openflex_all/openflex_ws/src/openarmx_teleop_exo-6.0_basic` yet. This dependency can be removed only after the WebSocket receiver is migrated into this package.

## Build

```bash
cd ~/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openflex_teleop_exo
source install/setup.bash
```

## Recommended Launch Order

Start the OpenFlex full-machine bringup first.

Simulation:

```bash
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py use_fake_hardware:=true use_rviz:=true
```

Hardware:

```bash
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py use_fake_hardware:=false use_rviz:=true auto_homing:=true
```

Start the WebSocket data publisher separately:

```bash
ros2 launch openflex_teleop_exo websocket_publisher.launch.py
```

Confirm EXO data:

```bash
ros2 topic echo /exo/gamepad_keys
ros2 topic echo /exo/joint_command --once
```

Then start the OpenFlex EXO mapping chain:

```bash
ros2 launch openflex_teleop_exo exo_retargeting.launch.py
```

For simulation debugging, if the initial arm pose difference is too large, temporarily disable the dual-arm safety check:

```bash
ros2 launch openflex_teleop_exo exo_retargeting.launch.py enable_safety_check:=false
```

Do not disable safety checking on real hardware by default.

## EXO Button And Axis Mapping

EXO `/exo/gamepad_keys` uses `sensor_msgs/msg/Joy`:

- `axes[0]`: left joystick left/right, left is `1.0`, right is `-1.0`.
- `axes[1]`: left joystick up/down, down is `-1.0`, up is `1.0`.
- `axes[2]`: right joystick left/right, left is `1.0`, right is `-1.0`.
- `axes[3]`: right joystick up/down, currently not used for whole-body control.
- `buttons[5]`: left red A, lift column down.
- `buttons[6]`: left blue B, lift column up.
- `buttons[11]`: right red A, head down.
- `buttons[12]`: right blue B, head up.
- `buttons[13]`: right white C, head left.
- `buttons[14]`: right black D, head right.

Buttons are active-low: unpressed is `1`, pressed is `0`.

## Output Topics

- `/cmd_vel`: `geometry_msgs/msg/Twist`, chassis velocity.
- `/lift_manual_position_controller/jog_command`: `std_msgs/msg/Float64`, lift-column jog velocity command.
- `/head_forward_position_controller/commands`: `std_msgs/msg/Float64MultiArray`, head `[yaw, pitch]` position command.
- `/left_forward_position_controller/commands`: `std_msgs/msg/Float64MultiArray`, left arm 7 joints plus gripper.
- `/right_forward_position_controller/commands`: `std_msgs/msg/Float64MultiArray`, right arm 7 joints plus gripper.
- `/openflex_teleop_exo/speed_scale`: `std_msgs/msg/Float32`, speed scale.
- `/left_arm/joint_command`: retargeted left-arm JointState.
- `/right_arm/joint_command`: retargeted right-arm JointState.

## Speed Parameters

Main parameters are in `config/openflex_teleop_exo.yaml`:

- `base_max_linear_speed: 1.0`: base chassis maximum linear speed.
- `base_max_angular_speed: 2.0`: base chassis maximum angular speed.
- `base_lift_speed: 0.5`: base lift-column jog speed.
- `base_head_rate_deg_s: 90.0`: base head angular speed, in deg/s.
- `default_speed_scale: 0.1`: default speed scale.

The current implementation multiplies chassis, lift column, and head speed by `speed_scale`. For example, the default effective head speed is:

```text
90.0 deg/s * 0.1 = 9.0 deg/s
```

## Dual-Arm Safety Handover Logic

- Wait for `/joint_states` and read the current left/right arm joint positions.
- When the first EXO dual-arm target is received, calculate the per-joint difference between the current robot pose and target pose.
- If the maximum difference is below `max_joint_diff_rad`, default `0.873 rad`, start interpolated handover over `interpolation_duration` seconds.
- If the maximum difference exceeds the threshold, print the failed joint, current value, target value, and difference, then stop the bridge node.

Available parameters:

```bash
ros2 launch openflex_teleop_exo exo_retargeting.launch.py max_joint_diff_rad:=1.5
ros2 launch openflex_teleop_exo exo_retargeting.launch.py interpolation_duration:=6.0
ros2 launch openflex_teleop_exo exo_retargeting.launch.py enable_safety_check:=false
```

On real hardware, keep `enable_safety_check:=true` and adjust the initial EXO posture so the difference is within the threshold.

## Lift Column Test

The lift column receives jog velocity commands, not absolute positions:

- Positive: move up.
- Negative: move down.
- `0.0`: stop.

Observe commands:

```bash
ros2 topic echo /lift_manual_position_controller/jog_command
```

Check controller state:

```bash
ros2 control list_controllers | grep lift
ros2 topic echo /lift_slide_driver/motor_status --once
ros2 topic echo /lift_slide_driver/limit_switch_state --once
```

On hardware, if commands are published but the lift does not move, focus on `is_enabled`, `homing_complete`, and limit-switch state.

Manual enable and homing:

```bash
ros2 service call /lift_slide_driver/enable std_srvs/srv/Trigger
ros2 service call /lift_slide_driver/start_homing std_srvs/srv/Trigger
```

## Common Debug Commands

```bash
ros2 node list | grep openflex_exo_teleop_node
ros2 topic info /lift_manual_position_controller/jog_command --verbose
ros2 topic echo /left_arm/joint_command --once
ros2 topic echo /right_arm/joint_command --once
ros2 topic echo /left_forward_position_controller/commands --once
ros2 topic echo /right_forward_position_controller/commands --once
ros2 topic echo /head_forward_position_controller/commands
ros2 topic echo /openflex_teleop_exo/speed_scale
```

## Safety Notes

- Before the first real-hardware test, confirm emergency stop, chassis, lift column, and dual-arm controller states.
- Do not disable dual-arm safety checking by default on real hardware.
- Before lift-column testing, confirm `is_enabled: true` and `homing_complete: true`.
- If the same topic has multiple publishers, stop duplicate launch instances first.

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenArmX robotic platform ecosystem, developed for research and industrial applications in collaborative robotics.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 🌐 Documentation  | [http://docs.openarmx.com/](http://docs.openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
