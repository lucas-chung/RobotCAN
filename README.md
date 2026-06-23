# RobotCAN

RobotCAN links TSMaster, Python, and MuJoCo for robot consistency testing.

Current demo stack:

- UR5e arm
- Robotiq 2F-85 gripper
- TSMasterAPI / CAN FD
- MuJoCo 3.9.0
- Modular Python test architecture

## Architecture

RobotCAN is intended as a modular test platform, not a single hard-coded demo.

Design goals:

- change algorithms without rewriting the task runner
- change robot models without rewriting perception or evaluation
- change simulation or real execution backends without rewriting policies

Runtime flow:

1. A task runner starts or resets a test episode.
2. MuJoCo renders observations such as RGB, depth, and robot state.
3. `perception` converts raw sensor data into structured objects.
4. `policy` decides the next goal from the observation.
5. `planning` converts the goal into robot actions such as joints or gripper targets.
6. `controller` / bridge executes the action through MuJoCo and/or TSMaster CAN FD.
7. `evaluation` records success, error, timing, and repeatability metrics.

## Repository Layout

- `main.py`: command entry
- `ENVIRONMENT.md`: local environment notes
- `requirements.txt`: Python package pins
- `dbc/`: DBC files
- `models/mujoco/`: MuJoCo scenes and assets
- `src/robotcan/core/`: shared types and interfaces
- `src/robotcan/app/`: CLI commands
- `src/robotcan/transport/`: TSMaster attach/send/receive layer
- `src/robotcan/protocol/`: CAN FD message definitions and codecs
- `src/robotcan/controller/`: bridge/controller loop
- `src/robotcan/simulation/`: MuJoCo backend and camera preview helpers
- `src/robotcan/perception/`: RGB-D perception and projection
- `src/robotcan/policies/`: task decision policies
- `src/robotcan/planning/`: future IK, grasp, and trajectory planning
- `src/robotcan/tasks/`: task orchestration
- `src/robotcan/evaluation/`: result summaries and future metrics

Extension boundaries:

- add a detector under `perception/` when the sensor algorithm changes
- add a policy under `policies/` when task decision logic changes
- add a planner under `planning/` when IK, trajectory, or robot kinematics changes
- swap MuJoCo XML and robot configuration when testing a different arm

## Current Capabilities

- attach to an opened TSMaster instance through RPC
- start and check TSMaster simulation from Python
- send cyclic or direct CAN FD control frames
- receive CAN FD FIFO frames
- bridge TSMaster CAN FD commands to a MuJoCo robot model
- simulate a combined UR5e + Robotiq 2F-85 scene
- preview a gripper-mounted RGB-D camera
- use a fixed top RGB-D camera for perception debugging
- detect a red target block and project pixel/depth into MuJoCo world coordinates
- run a simple Python-side algorithm demo observable in TSMaster

## Environment

This workspace uses a project-local virtual environment:

```powershell
.\.venv\Scripts\python.exe
```

Install dependencies:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Check installed packages:

```powershell
.\.venv\Scripts\python.exe -m pip show tsmaster.lib
.\.venv\Scripts\python.exe -m pip show mujoco
```

## Common Commands

Run commands from the repository root:

```powershell
cd path\to\RobotCAN
```

Check TSMaster status:

```powershell
.\.venv\Scripts\python.exe .\main.py status --app-name robot --server-name TSMaster
```

Start TSMaster simulation:

```powershell
.\.venv\Scripts\python.exe .\main.py start --app-name robot --server-name TSMaster
```

Run the bridge:

```powershell
.\.venv\Scripts\python.exe .\main.py bridge --app-name robot --server-name TSMaster --model .\models\mujoco\ur5e_2f85_scene.xml
```

Run the bridge with gripper camera RGB-D preview:

```powershell
.\.venv\Scripts\python.exe .\main.py bridge --app-name robot --server-name TSMaster --model .\models\mujoco\ur5e_2f85_scene.xml --camera-preview --camera-rgbd
```

Preview a MuJoCo camera without TSMaster:

```powershell
.\.venv\Scripts\python.exe .\main.py camera-preview --model .\models\mujoco\ur5e_2f85_scene.xml --camera-name gripper_depth_camera --rgbd
```

Run the perception demo:

```powershell
.\.venv\Scripts\python.exe .\main.py vision-detect-demo --model .\models\mujoco\ur5e_2f85_scene.xml
```

Run the repeated MuJoCo pick/place test:

```powershell
.\.venv\Scripts\python.exe .\main.py repeat-pick-place-test --model .\models\mujoco\ur5e_2f85_scene.xml --cycles 10
```

Run this command by itself. If a `bridge` command is already running with its
own MuJoCo viewer, close it first or use `--no-viewer`; otherwise two MuJoCo
windows will be open.

This test is still a deterministic MuJoCo test fixture. It uses simple
site-position IK to move the gripper down to the block, then only attaches the
block when the pinch site is close enough to the grasp pose. The defaults are
tuned to make the visual grasp happen near the block top. If the local model or
viewer angle still shows a gap, tune the fixture without editing code:

```powershell
.\.venv\Scripts\python.exe .\main.py repeat-pick-place-test --model .\models\mujoco\ur5e_2f85_scene.xml --cycles 10 --grasp-z 0.078 --attach-tolerance 0.018 --carry-offset 0.035
```

Replace the attach fixture with contact-based grasping and visual servoing as
those modules mature.

Run the Python-side algorithm demo:

```powershell
.\.venv\Scripts\python.exe .\main.py algorithm-demo --app-name robot --server-name TSMaster
```

Send hand control manually:

```powershell
.\.venv\Scripts\python.exe .\main.py send-control --app-name robot --server-name TSMaster --position 30 --velocity 0 --torque 0 --mode 1 --period-ms 10
```

Send arm control manually:

```powershell
.\.venv\Scripts\python.exe .\main.py send-arm-control --app-name robot --server-name TSMaster --j1 -90 --j2 -90 --j3 90 --j4 -90 --j5 -90 --j6 0 --mode 1 --period-ms 10
```

Clear cyclic CAN FD messages:

```powershell
.\.venv\Scripts\python.exe .\main.py clear-cyclic --app-name robot --server-name TSMaster
```

## CAN Message Split

Hand:

- `0x101` `Hand_Control_Command`
- `0x201` `Hand_Status_Feedback`
- `0x301` `Hand_Fault_Status`
- `0x401` `Hand_Diag_Response`

Arm:

- `0x110` `Arm_Control_Command`
- `0x210` `Arm_Status_Feedback`
- `0x310` `Arm_Fault_Status`
- `0x410` `Arm_Diag_Response`

## Vision-Grasp Roadmap

Implemented now:

- `top_camera` renders RGB-D observations
- `perception/color.py` detects the red target block
- `perception/rgbd.py` converts pixel + depth into MuJoCo world coordinates
- `tasks/vision_detect_demo.py` displays the detection result and prints target coordinates
- `tasks/repeat_pick_place_test.py` runs 10-cycle home-to-rotated-target pick/place testing

Next steps:

1. add a `visual_pick` policy that turns detected objects into grasp goals
2. add an IK/grasp planner that converts end-effector goals into joint targets
3. add repeatable test episodes with object randomization and metrics
4. add detector/policy variants for YOLO, visual servoing, RL, or LLM-driven logic
