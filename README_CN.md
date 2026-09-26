# openflex_EXO

[English](./README.md) | 中文

---

![封面](./image/cover.gif)


`openflex_EXO` 是 OpenFlex 外骨骼遥操作相关源码集合，用于接收外骨骼端数据，并将其转换为 OpenFlex 双臂或整机可用的遥操作控制输入。

## 目录结构

- `openflex_teleop_exo`：外骨骼遥操作主包，包含节点、配置和启动文件。
- `openflex_tool/openflex_teleop_exo_speed_panel`：外骨骼遥操作速度调节面板。

## 使用方式

编译工作空间后，先加载环境：

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
```

具体启动命令以 `openflex_teleop_exo/launch` 中的 launch 文件为准。速度参数和映射参数优先查看 `openflex_teleop_exo/config`。

## 说明

该目录主要面向外骨骼遥操作链路调试。若只使用 VR、GUI 或普通整机启动，可以不单独启动本目录下的节点。

## 许可证

本包通过 知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenFlex 全身人形机器人平台生态系统的一部分，专为人形机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | https://openarmx.com/ |
| 🌐 文档 | http://docs.openarmx.com/ |
| 📍 地址 | 天津市西青区・稻潮机器人体验基地（明日之城）・天津市人形机器人中心 |
| 👤 联系人 | 王先生 |
