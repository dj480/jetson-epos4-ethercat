import ctypes
import os
import time

# Load the compiled shared library from the current directory
lib_path = os.path.abspath("./libmotor.so")
motor = ctypes.CDLL(lib_path)

# Map C function signatures for ctypes safety
motor.motor_init.argtypes = [ctypes.c_char_p]
motor.motor_init.restype = ctypes.c_int

motor.motor_enable.argtypes = []
motor.motor_enable.restype = ctypes.c_int

motor.motor_disable.argtypes = []
motor.motor_disable.restype = ctypes.c_int

motor.motor_set_velocity.argtypes = [ctypes.c_uint32]
motor.motor_set_velocity.restype = ctypes.c_int

motor.motor_move_relative.argtypes = [ctypes.c_int32]
motor.motor_move_relative.restype = ctypes.c_int

motor.motor_get_position.argtypes = []
motor.motor_get_position.restype = ctypes.c_int32

motor.motor_close.argtypes = []
motor.motor_close.restype = None


def main():
    # 1. Initialize EtherCAT network on enP8p1s0
    print("Initializing EtherCAT interface...")
    if motor.motor_init(b"enP8p1s0") <= 0:
        print("Initialization failed! Check hardware / privileges.")
        return

    try:
        # 2. Transition CiA-402 state to Enable Operation
        print("\nEnabling drive...")
        motor.motor_enable()

        # 3. Read position from 0x6064
        initial_pos = motor.motor_get_position()
        print(f"\nInitial Position: {initial_pos}")

        # 4. Set Profile Velocity (0x6081)
        speed = 5000
        print(f"Setting profile velocity to {speed}...")
        motor.motor_set_velocity(speed)

        # 5. Execute relative move (+1000 counts)
        print("Executing move (+1000 counts)...")
        motor.motor_move_relative(1000)

        # 6. Read new position
        new_pos = motor.motor_get_position()
        print(f"Position after move: {new_pos}")

    finally:
        # 7. Clean up and close socket safely
        print("\nShutting down drive and closing EtherCAT context...")
        motor.motor_close()


if __name__ == "__main__":
    main()