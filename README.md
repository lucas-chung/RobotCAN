# RobotCAN

## English Version

### 1. Overview

RobotCAN is a lightweight robot CAN FD test project built around:

- `TSMaster`
- `Python`
- `MuJoCo`
- `TSMasterAPI`

The current version focuses on an attach-to-existing-TSMaster workflow:

- attach to an opened TSMaster project through RPC
- send and receive CAN FD messages through TSMasterAPI
- bridge CAN commands into MuJoCo
- run a combined `UR5e + Robotiq 2F-85` simulation model

### 1.1 Current Repository Structure

This repository intentionally keeps a shallow structure for easier browsing:

- `main.py`: Python entrypoint
- `ENVIRONMENT.md`: bilingual environment and version guide
- `dbc/`: DBC files
- `models/mujoco/`: MuJoCo scenes and robot assets
- `src/robotcan/`: source code

### 1.2 Current Status

The project currently supports:

- TSMaster RPC start/status/stop workflow
- cyclic CAN FD control frame creation
- CAN FD feedback readback
- TSMaster-to-MuJoCo bridge
- `UR5e + Robotiq 2F-85` model loading in MuJoCo

### 1.3 Quick Start

1. Open TSMaster and load your project
2. Import the DBC file in `dbc/`
3. Start simulation in TSMaster or use the Python CLI to start it
4. Run the bridge:

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py bridge --app-name robot --server-name TSMaster --model D:\data\RobotCAN\models\mujoco\ur5e_2f85_scene.xml
```

5. Send `0x101 Joint_Control_Command` from TSMaster

At the current stage, the existing single-command frame is mainly mapped to gripper open/close behavior.

### 1.4 Notes

- This repository currently uses a single control frame structure.
- Full 6-axis UR5e joint control plus gripper control will require a new CAN FD command design and DBC update.
- Verified environment details are documented in `ENVIRONMENT.md`.

---

## 中文版本

### 1. 项目概述

RobotCAN 是一个基于以下组件构建的轻量级机器人 CAN FD 测试项目：

- `TSMaster`
- `Python`
- `MuJoCo`
- `TSMasterAPI`

当前版本重点实现的是“连接已打开的 TSMaster”工作流：

- 通过 RPC 连接已经打开的 TSMaster 工程
- 通过 TSMasterAPI 发送和接收 CAN FD 报文
- 将 CAN 控制命令桥接到 MuJoCo
- 运行组合好的 `UR5e + Robotiq 2F-85` 仿真模型

### 1.1 当前仓库结构

为了方便大家点击查看，这个仓库尽量保持浅层目录：

- `main.py`：Python 入口
- `ENVIRONMENT.md`：中英文环境说明
- `dbc/`：DBC 文件
- `models/mujoco/`：MuJoCo 场景和机器人模型资源
- `src/robotcan/`：源码

### 1.2 当前功能状态

当前已经支持：

- TSMaster RPC 的启动 / 状态查询 / 停止
- 周期 CAN FD 控制报文创建
- CAN FD 反馈报文读取
- TSMaster 到 MuJoCo 的桥接
- 在 MuJoCo 中加载 `UR5e + Robotiq 2F-85`

### 1.3 快速开始

1. 打开 TSMaster 并加载工程
2. 导入 `dbc/` 下的 DBC 文件
3. 在 TSMaster 中启动仿真，或者使用 Python CLI 启动
4. 运行桥接：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py bridge --app-name robot --server-name TSMaster --model D:\data\RobotCAN\models\mujoco\ur5e_2f85_scene.xml
```

5. 在 TSMaster 中发送 `0x101 Joint_Control_Command`

当前阶段，这个单路控制报文主要映射为夹爪的开合控制。

### 1.4 说明

- 当前仓库使用的是单控制量报文结构
- 如果后续要完整控制 `UR5e` 六轴和夹爪，需要重新设计 CAN FD 控制报文和 DBC
- 已验证环境和安装方式请查看 `ENVIRONMENT.md`
