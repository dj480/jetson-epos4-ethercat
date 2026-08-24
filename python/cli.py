import ctypes
import os

# Resolve the shared library relative to the repository before trying the
# current working directory. The first two paths support the normal build
# layouts; the final path preserves compatibility with older local builds.
_PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
_LIB_CANDIDATES = [
    os.path.join(_PROJECT_ROOT, "lib", "libmotor.so"),
    os.path.join(_PROJECT_ROOT, "libmotor.so"),
    os.path.abspath("./libmotor.so"),
]

# Select the first existing candidate so importing this module fails with a
# useful message instead of producing an opaque ctypes loader error later.
lib_path = None
for p in _LIB_CANDIDATES:
    if os.path.exists(p):
        lib_path = p
        break

if lib_path is None:
    raise FileNotFoundError("libmotor.so not found in expected locations")

# Load the C API once. The resulting ``motor`` object is a ctypes proxy for
# the functions exported by the C shared library; it is not a Python motor
# implementation.
motor = ctypes.CDLL(lib_path)

# Declare argument and return types before making calls. Without these
# declarations ctypes may pass integers with the wrong width or interpret a
# native return value incorrectly, which is especially dangerous for counts
# and velocities.
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

# Continuous-motion functions are used by the pinch controller. The second
# argument to ``motor_start_continuous`` is the native update interval, not a
# Python sleep duration.
motor.motor_start_continuous.argtypes = [ctypes.c_int32, ctypes.c_uint32]
motor.motor_start_continuous.restype = ctypes.c_int

motor.motor_stop_continuous.argtypes = []
motor.motor_stop_continuous.restype = ctypes.c_int


def main():
    # Initialize EtherCAT before accepting commands.
    if motor.motor_init(b"enP8p1s0") <= 0:
        print("Initialization failed.")
        return

    # Start in an enabled state for interactive use.
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

            # Split commands from their optional numeric arguments uniformly.
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
