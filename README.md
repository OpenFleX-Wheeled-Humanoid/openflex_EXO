# openflex_EXO

English | [中文](./README_CN.md)

---

![Cover](./image/cover.gif)


`openflex_EXO` contains the OpenFlex exoskeleton teleoperation source packages. It receives exoskeleton-side data and converts it into teleoperation control inputs usable by the OpenFlex dual arms or whole-body system.

## Directory Layout

- `openflex_teleop_exo`: Main exoskeleton teleoperation package, including nodes, config files, and launch files.
- `openflex_tool/openflex_teleop_exo_speed_panel`: Speed adjustment panel for exoskeleton teleoperation.

## Usage

After building the workspace, source the environment:

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
```

Use the launch files under `openflex_teleop_exo/launch` for startup. For speed and mapping parameters, check `openflex_teleop_exo/config` first.

## Notes

This directory is mainly used for exoskeleton teleoperation chain debugging. If you only use VR, GUI, or the normal whole-body bringup, these nodes do not need to be launched separately.

## License

This package is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, please refer to the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenFlex full-body humanoid robot platform ecosystem, developed specifically for research and industrial applications in the humanoid robotics field.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.
**Chengdu Changshu Robotics Co., Ltd.**

| Contact | Information |
|---------|-------------|
| 📧 Email | openarmrobot@gmail.com |
| 📱 Phone/WeChat | +86-17746530375 |
| 🌐 Website | https://openarmx.com/ |
| 🌐 Docs | http://docs.openarmx.com/ |
| 📍 Address | Tianjin Xiqing District · Daochao Robot Experience Base (City of Tomorrow) · Tianjin Humanoid Robot Center |
| 👤 Contact Person | Mr. Wang |
