# Environment Setup

This file records the RobotCAN environment expected by the project.

## Verified Versions

- OS: Windows
- Python: 3.11+ recommended; this workspace was tested with Python 3.12.7
- TSMaster: 2026.5.13.1981
- `tsmaster.lib`: 2026.5.13.1981
- `mujoco`: 3.9.0

The original project was verified on Python 3.11.8. The current dependency set also installs and imports successfully on Python 3.12.7.

## Python Environment

Project root:

```powershell
<repo>
```

Virtual environment:

```powershell
<repo>\.venv
```

Python executable:

```powershell
.\.venv\Scripts\python.exe
```

## Install Dependencies

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Or install the two core packages directly:

```powershell
.\.venv\Scripts\python.exe -m pip install tsmaster.lib==2026.5.13.1981 mujoco==3.9.0
```

If a user-level pip mirror or proxy fails, temporarily ignore pip config and use official PyPI:

```powershell
$env:PIP_CONFIG_FILE='NUL'
.\.venv\Scripts\python.exe -m pip install --index-url https://pypi.org/simple -r requirements.txt
```

## Check Installed Packages

```powershell
.\.venv\Scripts\python.exe -m pip show tsmaster.lib
.\.venv\Scripts\python.exe -m pip show mujoco
```

## Before Running

Before starting the Python tools, make sure:

- TSMaster is installed
- TSMaster is already open
- the target TSMaster project is loaded
- the DBC has been imported
- TSMaster simulation can start normally

## Main Files

- entry: `main.py`
- CLI: `src/robotcan/app/cli.py`
- transport: `src/robotcan/transport/tsmaster_attached.py`
- controller loop: `src/robotcan/controller/loop.py`
- MuJoCo backend: `src/robotcan/simulation/mujoco/backend.py`
- current DBC: `dbc/robot_arm_hand_canfd.dbc`
- MuJoCo scene: `models/mujoco/ur5e_2f85_scene.xml`

## Common Commands

Run from the repository root:

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

Run bridge with RGB-D camera preview:

```powershell
.\.venv\Scripts\python.exe .\main.py bridge --app-name robot --server-name TSMaster --model .\models\mujoco\ur5e_2f85_scene.xml --camera-preview --camera-rgbd
```

Run perception demo:

```powershell
.\.venv\Scripts\python.exe .\main.py vision-detect-demo --model .\models\mujoco\ur5e_2f85_scene.xml
```

Send hand control:

```powershell
.\.venv\Scripts\python.exe .\main.py send-control --app-name robot --server-name TSMaster --position 30 --velocity 0 --torque 0 --mode 1 --period-ms 10
```

Send arm control:

```powershell
.\.venv\Scripts\python.exe .\main.py send-arm-control --app-name robot --server-name TSMaster --j1 0 --j2 -30 --j3 60 --j4 0 --j5 90 --j6 0 --mode 1 --period-ms 10
```
