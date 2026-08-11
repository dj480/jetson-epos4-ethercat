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