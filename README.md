# RobotCAN

## English

### 1. Overview

RobotCAN is a test project that links `TSMaster`, `Python`, and `MuJoCo`.

The current setup is:

- `TSMaster` works as the upper computer for CAN FD debugging and observation
- Python runs the bridge, demo tasks, and future algorithm logic
- `MuJoCo` acts as the simulated robot device

The current demo uses:

- `UR5e`
- `Robotiq 2F-85`
- `TSMasterAPI`
- CAN FD control and feedback

### 1.1 Current architecture

RobotCAN is being shaped as a modular robot consistency-test platform, not a
single hard-coded demo. The design goal is:

- change algorithms without rewriting the task runner
- change robot models without rewriting perception or evaluation
- change simulation/real execution backends without rewriting policies

The current high-level runtime roles are:

- `TSMaster`: CAN FD upper-computer debugging, observation, DBC behavior, and manual frame tools
- `Python`: task orchestration, perception, policies, planning, bridge logic, and evaluation
- `MuJoCo`: robot/device simulation, RGB-D cameras, scene objects, and robot state feedback

Typical data flow:

1. task runner resets or starts a test episode
2. MuJoCo renders observations such as RGB/depth images and robot state
3. `perception` converts raw sensor data into structured object information
4. `policy` decides the next goal from the current observation
5. `planning` converts goals into robot actions such as joint or gripper targets
6. `controller`/bridge executes the action through MuJoCo and/or TSMaster CAN FD
7. `evaluation` records success, error, timing, and repeatability metrics

### 1.2 Repository layout

This repository is kept fairly shallow so it is easy to browse in PyCharm or on GitHub:

- `main.py`: command entry
- `ENVIRONMENT.md`: environment notes and versions
- `dbc/`: DBC files
- `models/mujoco/`: MuJoCo scenes and assets
- `docs/`: project notes and helper material
- `src/robotcan/core/`: shared types and small interfaces (`Observation`, `DetectedObject`, `Policy`, `Planner`)
- `src/robotcan/app/`: CLI entry and app commands
- `src/robotcan/transport/`: TSMaster attach/send/receive layer
- `src/robotcan/protocol/`: CAN FD message definitions and codecs
- `src/robotcan/controller/`: bridge/controller logic
- `src/robotcan/simulation/`: MuJoCo backend, camera rendering, and preview helpers
- `src/robotcan/perception/`: RGB-D perception such as color detection and pixel/depth projection
- `src/robotcan/policies/`: decision policies, not raw image processing
- `src/robotcan/planning/`: future IK, grasp generation, and trajectory planning
- `src/robotcan/tasks/`: task orchestration
- `src/robotcan/evaluation/`: result summary and future metrics

The intended extension boundary is:

- add a new detector under `perception/` if the input sensor algorithm changes
- add a new policy under `policies/` if the task decision logic changes
- add a new planner under `planning/` if IK, trajectory, or robot kinematics changes
- add or swap MuJoCo XML and robot configuration when testing a different arm

This keeps the later path open for YOLO, GroundingDINO, SAM, visual servoing,
reinforcement learning, or multimodal/LLM policies without rewriting the whole
test loop.

### 1.3 What works now

Current code supports:

- attaching to an opened `TSMaster` instance through RPC
- starting and checking simulation from Python
- cyclic or direct CAN FD command transmission
- CAN FD FIFO receive
- a Python bridge between `TSMaster` and `MuJoCo`
- a combined `UR5e + Robotiq 2F-85` MuJoCo scene
- a gripper-mounted RGB-D camera preview
- a fixed top RGB-D camera for perception debugging
- red object detection with pixel/depth-to-world coordinate projection
- separate hand and arm control/feedback messages
- a simple Python-side demo task that can be observed in `TSMaster`

### 1.4 CAN message split

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

### 1.5 Main commands

Check TSMaster status:

```powershell
.\.venv\Scripts\python.exe .\RobotCAN-main\main.py status --app-name robot --server-name TSMaster
```

Start simulation:

```powershell
.\.venv\Scripts\python.exe .\RobotCAN-main\main.py start --app-name robot --server-name TSMaster
```

Run the bridge:

```powershell
.\.venv\Scripts\python.exe .\RobotCAN-main\main.py bridge --app-name robot --server-name TSMaster --model .\RobotCAN-main\models\mujoco\ur5e_2f85_scene.xml
```

Run the bridge with a gripper camera preview:

```powershell
.\.venv\Scripts\python.exe .\RobotCAN-main\main.py bridge --app-name robot --server-name TSMaster --model .\RobotCAN-main\models\mujoco\ur5e_2f85_scene.xml --camera-preview --camera-rgbd
```

Preview a MuJoCo camera without TSMaster:

```powershell
.\.venv\Scripts\python.exe .\RobotCAN-main\main.py camera-preview --model .\RobotCAN-main\models\mujoco\ur5e_2f85_scene.xml --camera-name gripper_depth_camera --rgbd
```

Run the current perception demo:

```powershell
.\.venv\Scripts\python.exe .\RobotCAN-main\main.py vision-detect-demo --model .\RobotCAN-main\models\mujoco\ur5e_2f85_scene.xml
```

Run the Python-side demo task:

```powershell
.\.venv\Scripts\python.exe .\RobotCAN-main\main.py algorithm-demo --app-name robot --server-name TSMaster
```

Send hand control manually:

```powershell
.\.venv\Scripts\python.exe .\RobotCAN-main\main.py send-control --app-name robot --server-name TSMaster --position 30 --velocity 0 --torque 0 --mode 1 --period-ms 10
```

Send arm control manually:

```powershell
.\.venv\Scripts\python.exe .\RobotCAN-main\main.py send-arm-control --app-name robot --server-name TSMaster --j1 -90 --j2 -90 --j3 90 --j4 -90 --j5 -90 --j6 0 --mode 1 --period-ms 10
```

### 1.6 Current vision-grasp roadmap

The current implemented step is perception only:

- `top_camera` renders RGB-D observations
- `perception/color.py` detects the red target block
- `perception/rgbd.py` converts pixel + depth into MuJoCo world coordinates
- `tasks/vision_detect_demo.py` displays the result and prints target coordinates

The next steps are:

1. add a `visual_pick` policy that turns detected objects into grasp goals
2. add an IK/grasp planner that converts end-effector goals into joint targets
3. add a repeatable test task with episode reset, object randomization, and CSV/JSONL metrics
4. add detector/policy variants for YOLO, visual servoing, or LLM-driven task logic

### 1.6 Recommended usage

Recommended daily workflow:

1. Keep `TSMaster` open as the upper computer
2. Use Python as the main control and algorithm side
3. Keep `MuJoCo` running as the simulated robot
4. Observe `Target_*` and `Actual_*` signals in `TSMaster`

In other words:

- `TSMaster` is for debugging, observation, and manual commands
- Python is for tasks, policies, and future algorithm evaluation
- `MuJoCo` is for execution and simulated feedback

---

## 中文

### 1. 项目说明

RobotCAN 是一个把 `TSMaster`、`Python` 和 `MuJoCo` 串起来的测试项目。

当前这套系统的定位是：

- `TSMaster` 作为上位机，负责看报文、发报文、看信号曲线
- Python 作为主控制和算法侧，负责 bridge、task、policy 和后续算法验证
- `MuJoCo` 作为被测仿真设备，负责机械臂、夹爪和场景执行

当前演示模型基于：

- `UR5e`
- `Robotiq 2F-85`
- `TSMasterAPI`
- CAN FD 控制与反馈

### 1.1 当前系统架构

这套工程可以简单理解成三层：

- `TSMaster`：上位机、调试台、观测台
- Python：主程序、bridge、任务执行器、后续算法入口
- `MuJoCo`：仿真机器人和场景

典型流程如下：

1. Python 或 `TSMaster` 发出控制命令
2. bridge 读取 CAN FD 控制帧
3. `MuJoCo` 执行机械臂和夹爪动作
4. bridge 回发状态、故障和诊断报文
5. `TSMaster` 显示报文、信号和图形

### 1.2 仓库结构

这个仓库尽量保持清晰，不把目录拆得太深，方便在 PyCharm 和 GitHub 里直接查看：

- `main.py`：命令入口
- `ENVIRONMENT.md`：环境、版本和安装说明
- `dbc/`：DBC 文件
- `models/mujoco/`：MuJoCo 场景和模型资源
- `docs/`：说明文档和辅助资料
- `src/robotcan/app/`：CLI 入口和命令定义
- `src/robotcan/transport/`：TSMaster 连接、收发报文相关
- `src/robotcan/protocol/`：CAN FD 报文定义和编解码
- `src/robotcan/controller/`：bridge/controller 核心逻辑
- `src/robotcan/simulation/`：MuJoCo backend
- `src/robotcan/policies/`：演示策略和后续算法策略
- `src/robotcan/tasks/`：任务流程组织
- `src/robotcan/evaluation/`：结果汇总和后续评估指标

### 1.3 当前已经完成的能力

当前代码已经支持：

- 通过 RPC 附着到已打开的 `TSMaster`
- 用 Python 查询和启动仿真
- 直接发送或周期发送 CAN FD 控制报文
- 从 FIFO 接收 CAN FD 报文
- 搭建 `TSMaster` 到 `MuJoCo` 的 Python bridge
- 跑通 `UR5e + Robotiq 2F-85` 联合场景
- 拆分 hand 和 arm 两套控制/反馈报文
- 用 Python 侧 demo task 发动作，并在 `TSMaster` 图形中观察目标值和实际值变化

### 1.4 当前报文划分

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

### 1.5 主要命令

查询 `TSMaster` 状态：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py status --app-name robot --server-name TSMaster
```

启动仿真：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py start --app-name robot --server-name TSMaster
```

运行 bridge：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py bridge --app-name robot --server-name TSMaster --model D:\data\RobotCAN\models\mujoco\ur5e_2f85_scene.xml
```

运行 Python 侧 demo task：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py algorithm-demo --app-name robot --server-name TSMaster
```

手工发送手爪控制命令：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-control --app-name robot --server-name TSMaster --position 30 --velocity 0 --torque 0 --mode 1 --period-ms 10
```

手工发送机械臂控制命令：

```powershell
D:\data\RobotCAN\RobotCAN\Scripts\python.exe D:\data\RobotCAN\main.py send-arm-control --app-name robot --server-name TSMaster --j1 -90 --j2 -90 --j3 90 --j4 -90 --j5 -90 --j6 0 --mode 1 --period-ms 10
```

### 1.6 推荐使用方式

目前推荐的使用方式是：

1. `TSMaster` 常开，作为上位机
2. Python 作为主控制和后续算法侧
3. `MuJoCo` 作为仿真执行环境
4. 在 `TSMaster` 中重点观察 `Target_*` 和 `Actual_*` 信号

也就是说：

- `TSMaster` 负责调试、观测、手工发命令
- Python 负责 task、policy、algorithm 和自动化验证
- `MuJoCo` 负责执行和回状态

### 1.7 下一步方向

当前阶段已经完成联调和信号观测。

接下来的主要方向是：

- 把 demo 动作升级成真正的 `pick task`
- 增加成功/失败判定
- 增加批量实验和成功率统计
- 后续把 `policies` 从脚本规则替换成更真实的算法或大模型
