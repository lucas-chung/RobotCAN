# 1. Environment Setup / 环境说明

## English Version

### 1.1 Project Overview

This project is a CAN FD robot test environment based on:

- `TSMaster`
- `Python`
- `MuJoCo`
- `TSMasterAPI`

The current implementation supports:

- attaching to an opened TSMaster instance through RPC
- sending and receiving CAN FD messages through TSMasterAPI
- running a Python bridge between TSMaster and MuJoCo
- loading a combined `UR5e + Robotiq 2F-85` MuJoCo scene

### 1.2 Verified Versions

- OS: `Windows`
- Python: `3.11.8`
- TSMaster: `2026.5.13.1981`
- `tsmaster.lib`: `2026.5.13.1981`
- `mujoco`: `3.9.0`

### 1.3 Python Virtual Environment

The current virtual environment name is `RobotCAN`.

Environment path:

- `D:\data\RobotCAN\RobotCAN`

### 1.4 Installation Commands

#### 1.4.1 Install TSMasterAPI Python package

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip install tsmaster.lib==2026.5.13.1981
```

#### 1.4.2 Install MuJoCo

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip install mujoco
```

#### 1.4.3 Check installed versions

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip show tsmaster.lib
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip show mujoco
```

### 1.5 TSMaster Runtime Requirements

Before running this project:

- install and open TSMaster
- use TSMaster version `2026.5.13.1981`
- open the target TSMaster project first
- import the DBC file
- make sure simulation can be started via RPC or is already running

### 1.6 Key Files

- Entrypoint: `main.py`
- CLI: `src/robotcan/app/cli.py`
- TSMaster transport: `src/robotcan/transport/tsmaster_attached.py`
- MuJoCo backend: `src/robotcan/simulation/mujoco/backend.py`
- DBC: `dbc/robot_joint_canfd.dbc`
- Single-joint scene: `models/mujoco/joint_scene.xml`
- UR5e + gripper scene: `models/mujoco/ur5e_2f85_scene.xml`

### 1.7 Common Commands

#### 1.7.1 Check TSMaster status

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py status --app-name robot --server-name TSMaster
```

#### 1.7.2 Start TSMaster simulation

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py start --app-name robot --server-name TSMaster
```

#### 1.7.3 Run the MuJoCo bridge

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py bridge --app-name robot --server-name TSMaster --model D:\data\RobotCAN\models\mujoco\ur5e_2f85_scene.xml
```

---

## 中文版本

### 1.1 项目概述

本项目是一个基于以下组件构建的 CAN FD 机器人测试环境：

- `TSMaster`
- `Python`
- `MuJoCo`
- `TSMasterAPI`

当前实现已经支持：

- 通过 RPC 连接已打开的 TSMaster 实例
- 通过 TSMasterAPI 发送和接收 CAN FD 报文
- 使用 Python 在 TSMaster 与 MuJoCo 之间做桥接
- 加载组合好的 `UR5e + Robotiq 2F-85` MuJoCo 场景

### 1.2 已验证版本

- 操作系统：`Windows`
- Python：`3.11.8`
- TSMaster：`2026.5.13.1981`
- `tsmaster.lib`：`2026.5.13.1981`
- `mujoco`：`3.9.0`

### 1.3 Python 虚拟环境

当前虚拟环境名称为 `RobotCAN`。

虚拟环境路径：

- `D:\data\RobotCAN\RobotCAN`

### 1.4 安装命令

#### 1.4.1 安装 TSMasterAPI 对应 Python 包

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip install tsmaster.lib==2026.5.13.1981
```

#### 1.4.2 安装 MuJoCo

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip install mujoco
```

#### 1.4.3 查看已安装版本

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip show tsmaster.lib
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip show mujoco
```

### 1.5 TSMaster 运行前提

在运行本项目之前，请确认：

- 已安装并打开 TSMaster
- 使用的 TSMaster 版本为 `2026.5.13.1981`
- 已提前打开目标 TSMaster 工程
- 已导入 DBC 文件
- 仿真可通过 RPC 启动，或已经处于运行状态

### 1.6 关键文件

- 入口：`main.py`
- CLI：`src/robotcan/app/cli.py`
- TSMaster 传输层：`src/robotcan/transport/tsmaster_attached.py`
- MuJoCo 后端：`src/robotcan/simulation/mujoco/backend.py`
- DBC：`dbc/robot_joint_canfd.dbc`
- 单关节场景：`models/mujoco/joint_scene.xml`
- 机械臂+夹爪场景：`models/mujoco/ur5e_2f85_scene.xml`

### 1.7 常用命令

#### 1.7.1 检查 TSMaster 状态

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py status --app-name robot --server-name TSMaster
```

#### 1.7.2 启动 TSMaster 仿真

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py start --app-name robot --server-name TSMaster
```

#### 1.7.3 运行 MuJoCo 桥接

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py bridge --app-name robot --server-name TSMaster --model D:\data\RobotCAN\models\mujoco\ur5e_2f85_scene.xml
```
