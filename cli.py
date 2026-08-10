import ctypes
import os

# Load shared library
lib_path = os.path.abspath("./libmotor.so")
motor = ctypes.CDLL(lib_path)

# Declare function signatures for ctypes safety
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
    # Initialize EtherCAT interface
    if motor.motor_init(b"enP8p1s0") <= 0:
        print("Initialization failed.")
        return

    # Enable drive automatically on start
    motor.motor_enable()

    try:
        while True:
            print("\nCommands:")
            print("  p           Print position")
            print("  m <counts>  Move relative")
            print("  v <speed>   Set velocity")
            print("  e           Enable drive")
            print("  d           Disable drive")
            print("  q           Quit")

            user_input = input("> ").strip().split()
            if not user_input:
                continue

            cmd = user_input[0].lower()

            if cmd == "p":
                pos = motor.motor_get_position()
                print(f"Position = {pos}")

            elif cmd == "v":
                if len(user_input) > 1:
                    speed = int(user_input[1])
                    motor.motor_set_velocity(speed)
                else:
                    print("Usage: v <speed>")

            elif cmd == "m":
                if len(user_input) > 1:
                    counts = int(user_input[1])
                    motor.motor_move_relative(counts)
                else:
                    print("Usage: m <counts>")

            elif cmd == "e":
                motor.motor_enable()

            elif cmd == "d":
                motor.motor_disable()

            elif cmd == "q":
                print("Exiting application...")
                break

            else:
                print("Unknown command")

    finally:
        motor.motor_close()


if __name__ == "__main__":
    main()