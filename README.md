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

### Step 2: Grant Python Network Prvileges
 - Ethercat requires raw Ethernet socket access. In order to not need to recompile with sudo every time you make a change, paste this into terminal: 
 - "sudo setcap cap_net_raw,cap_net_admin=eip $(readlink -f$(which python3))"

### Step 3: Configure VS Code Python Interpreter
1. Open VS Code in the project workspace folder.
2. Press Ctrl + Shift + P to open the Command Palette.
3. Type Python: Select Interpreter and select it.
4. Choose /usr/bin/python3 or /usr/bin/python3.12; really whatever version you have.