# openflex_teleop_exo_speed_panel

RViz2 panel for controlling the OpenFlex EXO teleoperation speed scale.

Chinese documentation: [README_zh.md](README_zh.md)

## Features

- RViz2 panel plugin.
- Slider range: `0.00 - 0.50`.
- Preset buttons: `10%`, `25%`, `50%`.
- Incoming values above `0.50` are clamped to `0.50`.
- Publishes `/openflex_teleop_exo/speed_scale`.
- Message type: `std_msgs/msg/Float32`.
- Subscribes to the same topic and updates the panel when an external node changes the scale.

## Build

```bash
cd ~/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openflex_teleop_exo_speed_panel
source install/setup.bash
```

If RViz still loads an old library, close all RViz instances, open a new terminal, and source the workspace again:

```bash
source /opt/ros/humble/setup.bash
source ~/openflex_all/openflex_ws/install/setup.bash
rviz2
```

## Load In RViz2

1. Open RViz2.
2. Select `Panels` -> `Add New Panel`.
3. Select `openflex_teleop_exo_speed_panel/SpeedScalePanel`.
4. Use the slider or preset buttons to adjust the speed scale.

## Usage With openflex_teleop_exo

Start the OpenFlex EXO control chain:

```bash
ros2 launch openflex_teleop_exo exo_retargeting.launch.py
```

Then adjust the scale in RViz2. Command-line verification:

```bash
ros2 topic echo /openflex_teleop_exo/speed_scale
ros2 topic pub /openflex_teleop_exo/speed_scale std_msgs/msg/Float32 "{data: 0.2}" --once
```

`openflex_exo_teleop_node` currently applies this scale to chassis, lift, and head speed.

## Plugin Details

Plugin XML:

```text
plugins/plugin_description.xml
```

Plugin class:

```text
openflex_teleop_exo_speed_panel/SpeedScalePanel
```

Installed library:

```text
install/openflex_teleop_exo_speed_panel/lib/libopenflex_teleop_exo_speed_panel.so
```

## Troubleshooting

If RViz reports:

```text
The class required for this panel could not be loaded
undefined symbol: _ZTV...
```

This usually means Qt moc did not process the header containing `Q_OBJECT`. The CMake target now explicitly includes the header:

```cmake
add_library(${PROJECT_NAME} SHARED
  src/speed_scale_panel.cpp
  include/openflex_teleop_exo_speed_panel/speed_scale_panel.hpp
)
```

Clean rebuild:

```bash
cd ~/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openflex_teleop_exo_speed_panel --cmake-clean-cache
source install/setup.bash
```

Check package discovery:

```bash
ros2 pkg prefix openflex_teleop_exo_speed_panel
```

Check speed-scale topic:

```bash
ros2 topic info /openflex_teleop_exo/speed_scale --verbose
```

If the panel loads but does not affect motion, confirm the teleop node is running:

```bash
ros2 node list | grep openflex_exo_teleop_node
```

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
