# openflex_teleop_exo

OpenFlex 外骨骼全身遥操作适配包。该包把 EXO 外骨骼数据转换为 OpenFlex 的底盘、升降柱、头部和双臂控制命令。

## 功能范围

- 接收 EXO WebSocket 发布的 `/exo/joint_command` 和 `/exo/gamepad_keys`。
- 双臂：`/exo/joint_command` 重定向到左右臂目标关节，再桥接到 OpenFlex 双臂 position controller。
- 底盘：EXO 左摇杆控制平移，右摇杆 X 控制旋转。
- 升降柱：EXO 左手红/蓝按键控制下降/上升。
- 头部：EXO 右手红/蓝/白/黑按键控制俯仰和左右转动。
- 速度倍率：订阅并发布 `/openflex_teleop_exo/speed_scale`，可配合 RViz 面板动态调节。

## 包结构

- `openflex_exo_teleop_node.py`：底盘、升降柱、头部和速度倍率控制。
- `arm_retargeting_node.py`：EXO 双臂关节到 OpenFlex 双臂关节的重定向。
- `arm_bridge_node.py`：双臂安全检查、渐进接管和 controller 命令发布。
- `config/openflex_teleop_exo.yaml`：全身控制参数。
- `config/retargeting_OpenArmX.yaml`：双臂重定向参数。
- `launch/websocket_publisher.launch.py`：启动 EXO WebSocket 数据接收发布节点。
- `launch/exo_retargeting.launch.py`：启动 OpenFlex EXO 映射和控制链路。
- `launch/openflex_teleop_exo.launch.py`：安全默认启动，仅启动映射链路，不启动 WebSocket。
- `launch/openflex_full_exo_teleop.launch.py`：WebSocket 和映射链路一起启动。


## 编译

```bash
cd ~/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openflex_teleop_exo
source install/setup.bash
```

## 推荐启动顺序

### 先启动 OpenFlex 整机 bringup。

仿真:
```bash
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py use_fake_hardware:=true use_rviz:=true
```

硬件：

```bash
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py use_fake_hardware:=false use_rviz:=true auto_homing:=true
```

### 单独启动 WebSocket 数据发布：

```bash
ros2 launch openflex_teleop_exo websocket_publisher.launch.py
```


### 然后在上位机执行以下操作：

1. 打开上位机 `Qnbot HMI Control-1.2.2.AppImage`。
2. 点击左侧“外骨骼设备”。
3. 点击“添加外骨骼设备”，创建连接。
4. 在设备管理中点击小齿轮，将 `ws://localhost:19091` 或`ws://{IP}:19091`添加到转发目标。
5. 再次将 `ws://localhost:19091` 或`ws://{IP}:19091` 粘贴到输入框中，点击“应用配置”。

<p>
  <span style="color:#B00020; font-size:18px;"><b>⚠️ 关键检查：</b></span>
  <span style="color:#B00020;"><b>如果没看到外骨骼数据转发，请优先检查上位机里的 WebSocket 目标地址是否正确。</b></span>
  <span style="color:#B00020;"><b>注意：两个手柄上的开关必须全部推上去才能进行通信！单人操作时可通过开关控制何时开始操作！</b></span>
</p>
 

确认 EXO 数据：

```bash
ros2 topic echo /exo/gamepad_keys
ros2 topic echo /exo/joint_command --once
```

### 再启动 OpenFlex EXO 映射链路：

```bash
ros2 launch openflex_teleop_exo exo_retargeting.launch.py
```

仿真调试时如双臂初始姿态差异过大，可临时关闭双臂安全检查：

```bash
ros2 launch openflex_teleop_exo exo_retargeting.launch.py enable_safety_check:=false
```

真实硬件不建议关闭安全检查。

## EXO 按键和轴映射

EXO `/exo/gamepad_keys` 使用 `sensor_msgs/msg/Joy`：

- `axes[0]`：左手摇杆左右，左为 `1.0`，右为 `-1.0`。
- `axes[1]`：左手摇杆上下，下为 `-1.0`，上为 `1.0`。
- `axes[2]`：右手摇杆左右，左为 `1.0`，右为 `-1.0`。
- `axes[3]`：右手摇杆上下，目前未用于全身控制。
- `buttons[5]`：左手红色 A，下降升降柱。
- `buttons[6]`：左手蓝色 B，上升升降柱。
- `buttons[11]`：右手红色 A，头部向下。
- `buttons[12]`：右手蓝色 B，头部向上。
- `buttons[13]`：右手白色 C，头部向左。
- `buttons[14]`：右手黑色 D，头部向右。

按钮为 active-low：未按为 `1`，按下为 `0`。

## 输出话题

- `/cmd_vel`：`geometry_msgs/msg/Twist`，底盘速度。
- `/lift_manual_position_controller/jog_command`：`std_msgs/msg/Float64`，升降柱点动速度命令。
- `/head_forward_position_controller/commands`：`std_msgs/msg/Float64MultiArray`，头部 `[yaw, pitch]` 位置命令。
- `/left_forward_position_controller/commands`：`std_msgs/msg/Float64MultiArray`，左臂 7 关节 + 夹爪。
- `/right_forward_position_controller/commands`：`std_msgs/msg/Float64MultiArray`，右臂 7 关节 + 夹爪。
- `/openflex_teleop_exo/speed_scale`：`std_msgs/msg/Float32`，速度倍率。
- `/left_arm/joint_command`：重定向后的左臂 JointState。
- `/right_arm/joint_command`：重定向后的右臂 JointState。

## 速度参数

主要参数在 `config/openflex_teleop_exo.yaml`：

- `base_max_linear_speed: 1.0`：底盘最大线速度基础值。
- `base_max_angular_speed: 2.0`：底盘最大角速度基础值。
- `base_lift_speed: 0.5`：升降柱基础点动速度。
- `base_head_rate_deg_s: 90.0`：头部基础角速度，单位 deg/s。
- `default_speed_scale: 0.1`：默认速度倍率。

当前实现中底盘、升降柱和头部都会乘以 `speed_scale`。例如默认头部实际速度为：

```text
90.0 deg/s * 0.1 = 9.0 deg/s
```

## 双臂安全接管逻辑

- 等待 `/joint_states`，读取机器人当前左右臂关节位置。
- 第一次收到 EXO 双臂目标时，计算当前姿态与目标姿态的每关节差值。
- 若最大差值小于 `max_joint_diff_rad`，默认 `0.873 rad`，开始 `interpolation_duration` 秒插值接管。
- 若最大差值超过阈值，打印失败关节、当前值、目标值和差值，然后退出 bridge 节点。

可用参数：

```bash
ros2 launch openflex_teleop_exo exo_retargeting.launch.py max_joint_diff_rad:=1.5
ros2 launch openflex_teleop_exo exo_retargeting.launch.py interpolation_duration:=6.0
ros2 launch openflex_teleop_exo exo_retargeting.launch.py enable_safety_check:=false
```

真实硬件建议保持 `enable_safety_check:=true`，通过调整 EXO 初始姿态让差值进入阈值。

## 升降柱测试

升降柱接收的是点动速度命令，不是绝对位置：

- 正数：向上。
- 负数：向下。
- `0.0`：停止。

观察命令：

```bash
ros2 topic echo /lift_manual_position_controller/jog_command
```

查看控制器状态：

```bash
ros2 control list_controllers | grep lift
ros2 topic echo /lift_slide_driver/motor_status --once
ros2 topic echo /lift_slide_driver/limit_switch_state --once
```

硬件上若有命令但不动，重点检查 `is_enabled`、`homing_complete` 和限位开关状态。

手动使能和回零：

```bash
ros2 service call /lift_slide_driver/enable std_srvs/srv/Trigger
ros2 service call /lift_slide_driver/start_homing std_srvs/srv/Trigger
```

## 常用调试命令

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

## 安全建议

- 真实硬件首次测试时，先确认急停、底盘、升降柱和双臂控制器状态。
- 真实硬件不要默认关闭双臂安全检查。
- 升降柱测试前先确认 `is_enabled: true`、`homing_complete: true`。
- 若发现同一话题有多个发布者，先停止重复的 launch 实例。

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenArmX 机器人平台生态系统的一部分，专为协作机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 🌐 文档 | <http://docs.openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
