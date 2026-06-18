# 1. Environment Setup / 环境说明

## English

### 1.1 Purpose

This document records the environment used for the current RobotCAN demo.

It is mainly here so the setup can be reproduced without guessing versions.

### 1.2 Verified versions

- OS: `Windows`
- Python: `3.11.8`
- TSMaster: `2026.5.13.1981`
- `tsmaster.lib`: `2026.5.13.1981`
- `mujoco`: `3.9.0`

### 1.3 Python environment

The current virtual environment name is `RobotCAN`.

Path:

- `D:\data\RobotCAN\RobotCAN`

### 1.4 Install commands

#### 1.4.1 Install TSMasterAPI

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip install tsmaster.lib==2026.5.13.1981
```

#### 1.4.2 Install MuJoCo

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip install mujoco==3.9.0
```

#### 1.4.3 Check installed packages

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip show tsmaster.lib
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip show mujoco
```

### 1.5 Before running

Before starting the Python tools, make sure:

- `TSMaster` is installed
- `TSMaster` is already open
- the target project is already loaded
- the DBC has been imported
- simulation can start normally

### 1.6 Main files

- entry: `main.py`
- CLI: `src/robotcan/app/cli.py`
- transport: `src/robotcan/transport/tsmaster_attached.py`
- controller loop: `src/robotcan/controller/loop.py`
- MuJoCo backend: `src/robotcan/simulation/mujoco/backend.py`
- current DBC: `dbc/robot_arm_hand_canfd.dbc`
- MuJoCo demo scene: `models/mujoco/ur5e_2f85_scene.xml`

### 1.7 Common commands

#### 1.7.1 Check TSMaster status

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py status --app-name robot --server-name TSMaster
```

#### 1.7.2 Start TSMaster simulation

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py start --app-name robot --server-name TSMaster
```

#### 1.7.3 Run the bridge

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py bridge --app-name robot --server-name TSMaster --model D:\data\RobotCAN\models\mujoco\ur5e_2f85_scene.xml
```

#### 1.7.4 Send hand control

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-control --app-name robot --server-name TSMaster --position 30 --velocity 0 --torque 0 --mode 1 --period-ms 10
```

#### 1.7.5 Send arm control

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-arm-control --app-name robot --server-name TSMaster --j1 0 --j2 -30 --j3 60 --j4 0 --j5 90 --j6 0 --mode 1 --period-ms 10
```

---

## 中文

### 1.1 文档用途

这个文档主要是把当前 RobotCAN 演示环境记清楚。

后面谁来复现这套环境，都不用再一点点猜版本。

### 1.2 已验证版本

- 操作系统：`Windows`
- Python：`3.11.8`
- TSMaster：`2026.5.13.1981`
- `tsmaster.lib`：`2026.5.13.1981`
- `mujoco`：`3.9.0`

### 1.3 Python 虚拟环境

当前虚拟环境名称是 `RobotCAN`。

路径：

- `D:\data\RobotCAN\RobotCAN`

### 1.4 安装命令

#### 1.4.1 安装 TSMasterAPI

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip install tsmaster.lib==2026.5.13.1981
```

#### 1.4.2 安装 MuJoCo

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip install mujoco==3.9.0
```

#### 1.4.3 查看安装结果

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip show tsmaster.lib
D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip show mujoco
```

### 1.5 运行前确认

在跑 Python 工具前，先确认下面几件事：

- `TSMaster` 已经安装好
- `TSMaster` 已经打开
- 目标工程已经加载
- DBC 已经导入
- 仿真本身可以正常启动

### 1.6 关键文件

- 入口：`main.py`
- CLI：`src/robotcan/app/cli.py`
- 传输层：`src/robotcan/transport/tsmaster_attached.py`
- 控制循环：`src/robotcan/controller/loop.py`
- MuJoCo 后端：`src/robotcan/simulation/mujoco/backend.py`
- 当前 DBC：`dbc/robot_arm_hand_canfd.dbc`
- MuJoCo 演示场景：`models/mujoco/ur5e_2f85_scene.xml`

### 1.7 常用命令

#### 1.7.1 查看 TSMaster 状态

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py status --app-name robot --server-name TSMaster
```

#### 1.7.2 启动 TSMaster 仿真

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py start --app-name robot --server-name TSMaster
```

#### 1.7.3 运行桥接程序

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py bridge --app-name robot --server-name TSMaster --model D:\data\RobotCAN\models\mujoco\ur5e_2f85_scene.xml
```

#### 1.7.4 发送手爪控制

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-control --app-name robot --server-name TSMaster --position 30 --velocity 0 --torque 0 --mode 1 --period-ms 10
```

#### 1.7.5 发送机械臂控制

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-arm-control --app-name robot --server-name TSMaster --j1 0 --j2 -30 --j3 60 --j4 0 --j5 90 --j6 0 --mode 1 --period-ms 10
```
