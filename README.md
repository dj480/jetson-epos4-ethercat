# EPOS4 EtherCAT Python Interface

Python control interface for Maxon EPOS4 motor controllers built on C/SOEM via `ctypes`.

---

## Machine-Specific Configurations

Before building or running code, ensure these two settings match your local machine environment:

1. **Ethernet Network Interface:**
   - Run `ip a` in your terminal to list active network adapters.
   - If your Ethernet interface is not `enP8p1s0`, update the interface parameter string inside your Python script (e.g., `EPOS4(interface="your_interface_name")`).

2. **SOEM Library Path:**
   - The build script defaults to `/home/danielbetts/SOEM`.
   - If SOEM is installed under a different user or directory on your system, open `build.sh` and update all include `-I` paths and the `libsoem.a` static library path to reflect your actual location.
---

## Environment Setup Instructions

### Step 1: Make `build.sh` Executable & Compile
Compile the C shared library (`libmotor.so`) locally:
```bash
chmod +x build.sh
./build.sh
```

### Step 2: Grant Python Network Privileges
EtherCAT requires raw Ethernet socket access. To avoid running the Python
process with `sudo` each time, grant the selected Python interpreter the
required capabilities:
```bash
sudo setcap cap_net_raw,cap_net_admin=eip $(readlink -f $(which python3))
```

### Step 3: Configure VS Code Python Interpreter
1. Open VS Code in the project workspace folder.
2. Press Ctrl + Shift + P to open the Command Palette.
3. Type Python: Select Interpreter and select it.
4. Choose /usr/bin/python3 or /usr/bin/python3.12; really whatever version you have.

---

## Repository Map

The project is divided by responsibility: native motor control, Python
orchestration, runtime artifacts, and development/archive files.

### Root-level command entry points

- `main.py` starts the interactive motor CLI.
- `cli.py` is a compatibility entry point for that same CLI.
- `pinch_client.py` starts camera-only pinch event reporting.
- `pinch_motor_control.py` starts pinch-controlled motor motion.
- `README.md` contains setup, machine configuration, and this repository map.

These small root scripts forward to the implementations in `python/`, so a
user can run commands from the repository root without knowing the package
layout.

### `c_src/`: native EtherCAT implementation

This folder contains the C source and headers that communicate with the EPOS4
drive through SOEM. It is the hardware-facing layer: initialization,
enable/disable, velocity configuration, position reads, relative moves, and
continuous motion are implemented here. The build process compiles this code
into the shared library consumed by Python.

### `python/`: application and integration layer

This package contains the maintained Python implementations:

- `cli.py` loads `libmotor.so` with `ctypes`, declares the C function
   signatures, and exposes interactive motor commands.
- `jetson_pinch_service.py` captures camera frames, detects a pinch with
   MediaPipe or the OpenCV fallback, and emits gesture events.
- `pinch_motor_control.py` connects those events to motor direction and
   continuous movement on a worker thread.
- `pinch_client.py` prints pinch events without commanding the motor.

Python is intentionally above the C layer: vision and user interaction stay
in Python, while timing-sensitive EtherCAT operations remain in native code.

### `lib/`: runtime native libraries

This folder holds built shared libraries, especially `libmotor.so`. Python
loads the library from here at runtime. It is generated output rather than the
source of the motor behavior; changes to native behavior belong in `c_src/`.

### `bin/`: compiled utility executables

This folder contains command-line binaries produced from native sources, such
as `read_status` and `test_soem`. They are useful for checking the drive or
SOEM independently of the Python application.

### `models/`: machine-learning assets

This folder stores the MediaPipe hand-landmarker model used by the primary
pinch detector. The model is loaded only when MediaPipe Tasks is available;
otherwise the service can use its OpenCV contour fallback.

### `build/`: build outputs and preserved artifacts

`build/archives/` contains snapshots such as previous service scripts, build
logs, and backup libraries. These files help with investigation or recovery,
but they are not the active implementation used by the root commands.

### `.vscode/`: editor configuration

This folder contains workspace-specific VS Code settings, tasks, or launch
configuration when present. It affects local development ergonomics, not the
runtime motor protocol.