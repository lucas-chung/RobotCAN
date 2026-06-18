# RobotCAN

## English

### 1. What this repo is

RobotCAN is a small test project that connects `TSMaster`, `Python`, and `MuJoCo`.

The workflow is straightforward:

- `TSMaster` sends and monitors CAN FD frames
- the Python bridge attaches to the opened `TSMaster` project
- the bridge forwards control frames into `MuJoCo`
- `MuJoCo` runs the robot model and sends state back through the bridge

The current demo is built around:

- `UR5e`
- `Robotiq 2F-85`
- CAN FD control and feedback through `TSMasterAPI`

### 1.1 Repository layout

This repository is kept intentionally shallow so it is easy to open and browse:

- `main.py`: command entry
- `ENVIRONMENT.md`: environment and version notes
- `dbc/`: DBC files
- `models/mujoco/`: MuJoCo scenes and robot assets
- `src/robotcan/`: Python source

### 1.2 What works now

Current code already supports:

- attaching to an opened `TSMaster` instance through RPC
- checking and starting simulation from Python
- cyclic CAN FD transmission
- CAN FD FIFO receive
- a Python bridge from `TSMaster` to `MuJoCo`
- a combined `UR5e + Robotiq 2F-85` MuJoCo scene
- separate hand and arm command channels

### 1.3 Current CAN message split

#### Hand

- `0x101` `Hand_Control_Command`
- `0x201` `Hand_Status_Feedback`
- `0x301` `Hand_Fault_Status`
- `0x401` `Hand_Diag_Response`

#### Arm

- `0x110` `Arm_Control_Command`
- `0x210` `Arm_Status_Feedback`
- `0x310` `Arm_Fault_Status`
- `0x410` `Arm_Diag_Response`

### 1.4 Quick start

1. Open `TSMaster`
2. Open your target project
3. Import the DBC in `dbc/`
4. Make sure simulation can start normally
5. Run the bridge:

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py bridge --app-name robot --server-name TSMaster --model D:\data\RobotCAN\models\mujoco\ur5e_2f85_scene.xml
```

6. Send hand control:

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-control --app-name robot --server-name TSMaster --position 30 --velocity 0 --torque 0 --mode 1 --period-ms 10
```

7. Send arm control:

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-arm-control --app-name robot --server-name TSMaster --j1 0 --j2 -30 --j3 60 --j4 0 --j5 90 --j6 0 --mode 1 --period-ms 10
```

### 1.5 Notes

- `TSMaster` should already be open before running the Python tools
- this project uses attach mode and does not try to open a second `TSMaster` instance
- when the DBC changes, the Python protocol definitions should be updated together
- detailed setup notes are in `ENVIRONMENT.md`

---

## 中文

### 1. 这个仓库是做什么的

RobotCAN 是一个把 `TSMaster`、`Python` 和 `MuJoCo` 串起来的小型测试工程。

整套流程比较直接：

- `TSMaster` 负责发报文、看报文
- Python 桥接程序附着到已经打开的 `TSMaster` 工程
- 桥接程序把控制报文读出来，转给 `MuJoCo`
- `MuJoCo` 跑模型，再把状态通过桥接程序回到总线

目前这套演示主要围绕下面这组模型跑通：

- `UR5e`
- `Robotiq 2F-85`
- 基于 `TSMasterAPI` 的 CAN FD 控制与反馈

### 1.1 仓库结构

这个仓库故意没有搞很多层目录，方便直接点开看：

- `main.py`：命令入口
- `ENVIRONMENT.md`：环境和版本说明
- `dbc/`：DBC 文件
- `models/mujoco/`：MuJoCo 场景和模型资源
- `src/robotcan/`：Python 源码

### 1.2 现在已经能做什么

当前版本已经打通了这些事情：

- 通过 RPC 附着到已经打开的 `TSMaster`
- 用 Python 查询和启动仿真状态
- 创建周期 CAN FD 报文
- 从 FIFO 读取 CAN FD 报文
- 在 `TSMaster` 和 `MuJoCo` 之间做桥接
- 加载 `UR5e + Robotiq 2F-85` 联合场景
- 把手爪控制和机械臂控制拆成两套报文

### 1.3 当前报文划分

#### 手爪

- `0x101` `Hand_Control_Command`
- `0x201` `Hand_Status_Feedback`
- `0x301` `Hand_Fault_Status`
- `0x401` `Hand_Diag_Response`

#### 机械臂

- `0x110` `Arm_Control_Command`
- `0x210` `Arm_Status_Feedback`
- `0x310` `Arm_Fault_Status`
- `0x410` `Arm_Diag_Response`

### 1.4 快速开始

1. 打开 `TSMaster`
2. 打开目标工程
3. 导入 `dbc/` 里的 DBC
4. 确认仿真能正常启动
5. 运行桥接程序：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py bridge --app-name robot --server-name TSMaster --model D:\data\RobotCAN\models\mujoco\ur5e_2f85_scene.xml
```

6. 发送手爪控制：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-control --app-name robot --server-name TSMaster --position 30 --velocity 0 --torque 0 --mode 1 --period-ms 10
```

7. 发送机械臂控制：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-arm-control --app-name robot --server-name TSMaster --j1 0 --j2 -30 --j3 60 --j4 0 --j5 90 --j6 0 --mode 1 --period-ms 10
```

### 1.5 说明

- 跑 Python 工具前，`TSMaster` 需要先打开
- 这个工程走的是附着模式，尽量不再新起一个 `TSMaster`
- 后面如果继续改 DBC，Python 里的协议定义也要一起改
- 更细的安装和版本信息看 `ENVIRONMENT.md`
