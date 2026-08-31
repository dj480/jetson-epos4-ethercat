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

motor.motor_slave_count.argtypes = []
motor.motor_slave_count.restype = ctypes.c_int

# Every per-drive function takes a leading 1-based slave index matching its
# EtherCAT chain position (see motor_slave_count()/bin/scan_slaves), not
# anything configured in EPOS Studio.
motor.motor_enable.argtypes = [ctypes.c_int]
motor.motor_enable.restype = ctypes.c_int

motor.motor_disable.argtypes = [ctypes.c_int]
motor.motor_disable.restype = ctypes.c_int

motor.motor_fault_reset.argtypes = [ctypes.c_int]
motor.motor_fault_reset.restype = ctypes.c_int

motor.motor_set_velocity.argtypes = [ctypes.c_int, ctypes.c_uint32]
motor.motor_set_velocity.restype = ctypes.c_int

motor.motor_set_acceleration.argtypes = [ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32]
motor.motor_set_acceleration.restype = ctypes.c_int

motor.motor_set_position_limits.argtypes = [ctypes.c_int, ctypes.c_int32, ctypes.c_int32]
motor.motor_set_position_limits.restype = ctypes.c_int

motor.motor_move_relative.argtypes = [ctypes.c_int, ctypes.c_int32]
motor.motor_move_relative.restype = ctypes.c_int

motor.motor_move_absolute.argtypes = [ctypes.c_int, ctypes.c_int32]
motor.motor_move_absolute.restype = ctypes.c_int

motor.motor_get_position.argtypes = [ctypes.c_int]
motor.motor_get_position.restype = ctypes.c_int32

motor.motor_close.argtypes = []
motor.motor_close.restype = None

# Continuous-motion functions are used by the pinch controller. The final
# argument to ``motor_start_continuous`` is the native update interval, not a
# Python sleep duration.
motor.motor_start_continuous.argtypes = [ctypes.c_int, ctypes.c_int32, ctypes.c_uint32]
motor.motor_start_continuous.restype = ctypes.c_int

motor.motor_stop_continuous.argtypes = [ctypes.c_int]
motor.motor_stop_continuous.restype = ctypes.c_int


def main():
    # Initialize EtherCAT before accepting commands. This configures every
    # slave found on the bus; commands below pick which one to address.
    slave_count = motor.motor_init(b"enP8p1s0")
    if slave_count <= 0:
        print("Initialization failed.")
        return

    print(f"{slave_count} slave(s) detected.")

    try:
        while True:
            print("\nCommands (<slave> is 1-based, matching EtherCAT chain position):")
            print("  p <slave>              Print position")
            print("  m <slave> <counts>     Move relative")
            print("  v <slave> <speed>      Set velocity")
            print("  l <slave> <min> <max>  Set drive-enforced position limits (0x607D)")
            print("  f <slave>              Reset a fault")
            print("  e <slave>              Enable drive")
            print("  d <slave>              Disable drive")
            print("  q                      Quit")

            # Split commands from their optional numeric arguments uniformly.
            user_input = input("> ").strip().split()
            if not user_input:
                continue

            cmd = user_input[0].lower()

            if cmd == "q":
                print("Exiting application...")
                break

            if cmd not in ("p", "m", "v", "l", "f", "e", "d"):
                print("Unknown command")
                continue

            if len(user_input) < 2:
                print(f"Usage: {cmd} <slave> ...")
                continue

            try:
                slave = int(user_input[1])
            except ValueError:
                print("Slave must be an integer.")
                continue

            if cmd == "p":
                pos = motor.motor_get_position(slave)
                print(f"Slave {slave}: Position = {pos}")

            elif cmd == "v":
                if len(user_input) > 2:
                    speed = int(user_input[2])
                    motor.motor_set_velocity(slave, speed)
                else:
                    print("Usage: v <slave> <speed>")

            elif cmd == "m":
                if len(user_input) > 2:
                    counts = int(user_input[2])
                    motor.motor_move_relative(slave, counts)
                else:
                    print("Usage: m <slave> <counts>")

            elif cmd == "l":
                if len(user_input) > 3:
                    min_pos = int(user_input[2])
                    max_pos = int(user_input[3])
                    motor.motor_set_position_limits(slave, min_pos, max_pos)
                else:
                    print("Usage: l <slave> <min> <max>")

            elif cmd == "f":
                motor.motor_fault_reset(slave)

            elif cmd == "e":
                motor.motor_enable(slave)

            elif cmd == "d":
                motor.motor_disable(slave)

    finally:
        motor.motor_close()


if __name__ == "__main__":
    main()
