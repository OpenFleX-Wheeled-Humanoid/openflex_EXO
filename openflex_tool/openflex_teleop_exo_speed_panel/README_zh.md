# openflex_teleop_exo_speed_panel

OpenFlex EXO 遥操作速度倍率 RViz2 面板。该面板发布 `/openflex_teleop_exo/speed_scale`，用于动态调整 `openflex_teleop_exo` 的速度倍率。

## 功能

- RViz2 Panel 插件。
- 滑动条范围 `0.00 - 0.50`。
- 预设按钮：`10%`、`25%`、`50%`。
- 外部发布超过 `0.50` 的倍率时，面板会自动限制为 `0.50`。
- 发布话题：`/openflex_teleop_exo/speed_scale`。
- 消息类型：`std_msgs/msg/Float32`。
- 同时订阅该话题，外部修改速度倍率时面板会同步显示。

## 编译

```bash
cd ~/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openflex_teleop_exo_speed_panel
source install/setup.bash
```

如果 RViz 仍加载旧库，建议关闭所有 RViz 后重新打开终端并重新 source：

```bash
source /opt/ros/humble/setup.bash
source ~/openflex_all/openflex_ws/install/setup.bash
rviz2
```

## RViz2 中加载

1. 打开 RViz2。
2. 菜单选择 `Panels` -> `Add New Panel`。
3. 选择 `openflex_teleop_exo_speed_panel/SpeedScalePanel`。
4. 添加后即可通过滑动条和预设按钮调整速度倍率。

## 与 openflex_teleop_exo 配合

先启动 OpenFlex EXO 映射链路：

```bash
ros2 launch openflex_teleop_exo exo_retargeting.launch.py
```

然后在 RViz2 面板中调整倍率。也可以用命令行验证：

```bash
ros2 topic echo /openflex_teleop_exo/speed_scale
ros2 topic pub /openflex_teleop_exo/speed_scale std_msgs/msg/Float32 "{data: 0.2}" --once
```

`openflex_exo_teleop_node` 当前会使用该倍率缩放底盘、升降柱和头部速度。

## 插件信息

插件 XML：

```text
plugins/plugin_description.xml
```

插件类名：

```text
openflex_teleop_exo_speed_panel/SpeedScalePanel
```

库文件安装位置：

```text
install/openflex_teleop_exo_speed_panel/lib/libopenflex_teleop_exo_speed_panel.so
```

## 故障排查

如果 RViz 报错：

```text
The class required for this panel could not be loaded
undefined symbol: _ZTV...
```

通常是 Qt moc 没有正确生成。当前 CMake 已显式把带 `Q_OBJECT` 的头文件加入库目标：

```cmake
add_library(${PROJECT_NAME} SHARED
  src/speed_scale_panel.cpp
  include/openflex_teleop_exo_speed_panel/speed_scale_panel.hpp
)
```

处理步骤：

```bash
cd ~/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openflex_teleop_exo_speed_panel --cmake-clean-cache
source install/setup.bash
```

确认插件被 ament 索引发现：

```bash
ros2 pkg prefix openflex_teleop_exo_speed_panel
```

确认速度倍率话题有发布者：

```bash
ros2 topic info /openflex_teleop_exo/speed_scale --verbose
```

如果面板能加载但控制无效，确认 `openflex_exo_teleop_node` 正在运行：

```bash
ros2 node list | grep openflex_exo_teleop_node
```

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
