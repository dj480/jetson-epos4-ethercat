import argparse
import sys
import threading
import time

from .cli import motor
from .jetson_pose_service import JetsonPoseTracker

# When stdout isn't a TTY (piped to a log file, VS Code's run-file output,
# etc.), Python block-buffers print() by default, so status/debug lines sit
# invisible until a large buffer fills or the process exits. Line-buffer it
# so they show up as they happen, matching motor.c's setvbuf fix.
sys.stdout.reconfigure(line_buffering=True)


def _clamp(value, low, high):
    return max(low, min(high, value))


class _JointFollower:
    """Per-joint state: smooth a measured segment angle into a rate-limited
    absolute motor target for one slave. Mirrors ArmMotorController's
    angle-to-position mapping, factored out so it can run twice (once per
    arm segment) against a single shared pose tracker."""

    def __init__(
        self,
        slave,
        down_position,
        up_position,
        down_angle_deg,
        up_angle_deg,
        velocity,
        acceleration,
        deceleration,
        deadband_counts,
        min_command_interval,
        smoothing,
        debug=False,
    ):
        if down_position == up_position:
            raise ValueError("down_position and up_position must differ.")
        if up_angle_deg <= down_angle_deg:
            raise ValueError("up_angle_deg must be greater than down_angle_deg.")

        self.slave = int(slave)
        # Measured encoder counts at the down/up calibration points, in
        # whatever order they came out as - see ArmMotorController for why
        # min()/max() rather than down/up decide the clamp range.
        self.down_position = int(down_position)
        self.up_position = int(up_position)
        self.min_position = min(self.down_position, self.up_position)
        self.max_position = max(self.down_position, self.up_position)
        self.down_angle_deg = float(down_angle_deg)
        self.up_angle_deg = float(up_angle_deg)
        self.velocity = velocity
        self.acceleration = int(acceleration)
        self.deceleration = int(deceleration)
        self.deadband_counts = int(deadband_counts)
        self.min_command_interval = float(min_command_interval)
        self.smoothing = float(smoothing)
        self.debug = bool(debug)

        self.motor_enabled = False
        self._lock = threading.Lock()
        self._smoothed_angle = None
        self._target_position = None
        self._visible = False
        self._last_commanded_position = None
        self._last_command_time = 0.0

    def on_angle(self, angle_deg, visible):
        # Pose-tracker callback: record the latest measurement only. Native
        # calls happen from the motor thread via service(), not here.
        with self._lock:
            if not visible:
                self._visible = False
                return

            if self._smoothed_angle is None:
                self._smoothed_angle = angle_deg
            else:
                self._smoothed_angle = (
                    self.smoothing * angle_deg + (1.0 - self.smoothing) * self._smoothed_angle
                )

            ratio = (self._smoothed_angle - self.down_angle_deg) / (
                self.up_angle_deg - self.down_angle_deg
            )
            ratio = _clamp(ratio, 0.0, 1.0)
            target = self.down_position + ratio * (self.up_position - self.down_position)
            self._target_position = int(round(target))
            self._visible = True

    def service(self):
        """Issue at most one move command for this joint's slave, if the
        latest target has moved enough and enough time has passed."""
        with self._lock:
            visible = self._visible
            target = self._target_position

        if not (visible and target is not None):
            return

        target = _clamp(target, self.min_position, self.max_position)
        now = time.monotonic()
        moved_enough = (
            self._last_commanded_position is None
            or abs(target - self._last_commanded_position) >= self.deadband_counts
        )
        waited_enough = (now - self._last_command_time) >= self.min_command_interval
        if not (moved_enough and waited_enough):
            return

        if not self.motor_enabled:
            motor.motor_enable(self.slave)
            motor.motor_set_velocity(self.slave, self.velocity)
            motor.motor_set_acceleration(self.slave, self.acceleration, self.deceleration)
            self.motor_enabled = True

        motor.motor_move_absolute(self.slave, target)
        self._last_commanded_position = target
        self._last_command_time = now

        if self.debug:
            print(f"[DEBUG] Slave {self.slave} moving to position {target}")

    def disable(self):
        if self.motor_enabled:
            motor.motor_disable(self.slave)
            self.motor_enabled = False


class TwoJointArmController:
    """Drive two EPOS4 drives from one arm at once: the upper-arm drive
    follows the shoulder->elbow segment, the forearm drive follows the
    elbow->wrist segment. Both segments come from one shared camera/pose
    pipeline (see JetsonPoseTracker's track_wrist option), so only one
    MediaPipe inference runs per frame regardless of how many joints are
    being followed.
    """

    def __init__(
        self,
        upper,
        lower,
        interface="enP8p1s0",
        camera_id=0,
        arm_side="right",
        preview=False,
        frame_width=640,
        frame_height=480,
    ):
        self.upper = upper
        self.lower = lower
        self.interface = interface
        self.camera_id = camera_id
        self.arm_side = arm_side
        self.preview = bool(preview)
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)

        self.tracker = None
        self.motor_thread = None
        self.running = False

    def start(self):
        # Initialize the native EtherCAT layer first and leave both drives
        # disabled until real pose measurements produce targets, matching
        # ArmMotorController's start-up ordering. Each step prints before
        # and after so a hang or failure is visible immediately instead of
        # showing up as silence.
        print("[1/4] Initializing EtherCAT and configuring drives...")
        slave_count = motor.motor_init(self.interface.encode("utf-8"))
        if slave_count <= 0:
            raise RuntimeError(f"Failed to initialize motor interface: {self.interface}")
        print(f"      {slave_count} slave(s) configured.")

        print(f"[2/4] Disabling slave {self.upper.slave} and slave {self.lower.slave} "
              f"until a pose measurement is available...")
        motor.motor_disable(self.upper.slave)
        motor.motor_disable(self.lower.slave)

        print("[3/4] Starting camera and pose tracker...")
        self.tracker = JetsonPoseTracker(
            camera_id=self.camera_id,
            arm_side=self.arm_side,
            track_wrist=True,
            on_pose_event=self.upper.on_angle,
            on_lower_pose_event=self.lower.on_angle,
            enable_preview=self.preview,
            frame_width=self.frame_width,
            frame_height=self.frame_height,
        )
        self.tracker.start()

        print("[4/4] Starting motor control loop...")
        self.running = True
        self.motor_thread = threading.Thread(target=self._motor_loop, daemon=True)
        self.motor_thread.start()

        print(
            "\nController running. Move your arm now.\n"
            "Press Ctrl+C (or close the preview window) to stop safely - "
            "both drives will be disabled before the process exits.\n"
        )

    def stop(self):
        print("\nStopping controller...")
        self.running = False

        print("[1/4] Stopping camera and pose tracker...")
        if self.tracker:
            try:
                self.tracker.stop()
            except Exception as e:
                print(f"[TwoJointArmController] Error stopping tracker: {e}")

        print("[2/4] Stopping motor control loop...")
        if self.motor_thread and self.motor_thread.is_alive():
            try:
                self.motor_thread.join(timeout=3.0)
            except Exception as e:
                print(f"[TwoJointArmController] Error joining motor thread: {e}")

        print(f"[3/4] Disabling slave {self.upper.slave} and slave {self.lower.slave}...")
        for joint in (self.upper, self.lower):
            try:
                joint.disable()
            except Exception as e:
                print(f"[TwoJointArmController] Error disabling slave {joint.slave}: {e}")

        print("[4/4] Closing EtherCAT...")
        try:
            motor.motor_close()
        except Exception as e:
            print(f"[TwoJointArmController] Error closing motor library: {e}")

        print("Shutdown complete - safe to exit.")

    def _motor_loop(self):
        # One thread services both joints; each service() call is a single
        # SDO round trip (or none, if that joint's deadband/rate limit isn't
        # satisfied yet), so interleaving them here does not starve either.
        while self.running:
            self.upper.service()
            self.lower.service()
            time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Track both arm segments (shoulder->elbow and elbow->wrist) and drive "
            "two EPOS4 drives to follow them at the same time."
        )
    )
    parser.add_argument("--camera-id", type=int, default=0, help="Camera index for video capture.")
    parser.add_argument("--interface", type=str, default="enP8p1s0", help="EtherCAT interface name.")
    parser.add_argument("--arm-side", choices=["left", "right"], default="right", help="Which arm to track.")
    parser.add_argument("--deadband-counts", type=int, default=20, help="Minimum position change before a new move.")
    parser.add_argument(
        "--min-command-interval", type=float, default=0.1, help="Minimum seconds between absolute-move commands."
    )
    parser.add_argument(
        "--smoothing", type=float, default=0.3, help="Exponential moving average factor for the measured angle."
    )
    parser.add_argument("--preview", action="store_true", help="Show a preview window with overlay for debugging.")
    parser.add_argument("--frame-width", type=int, default=640, help="Preview frame width.")
    parser.add_argument("--frame-height", type=int, default=480, help="Preview frame height.")

    parser.add_argument(
        "--upper-slave", type=int, default=1, help="EtherCAT chain position of the shoulder->elbow (upper-arm) drive."
    )
    parser.add_argument(
        "--upper-down-position",
        type=int,
        default=2000,
        help="Encoder position at the upper-arm 'down' calibration point.",
    )
    parser.add_argument(
        "--upper-up-position", type=int, default=12000, help="Encoder position at the upper-arm 'up' calibration point."
    )
    parser.add_argument("--upper-down-angle-deg", type=float, default=-30.0)
    parser.add_argument("--upper-up-angle-deg", type=float, default=60.0)
    parser.add_argument("--upper-velocity", type=int, default=5000)
    parser.add_argument("--upper-acceleration", type=int, default=5000)
    parser.add_argument("--upper-deceleration", type=int, default=5000)

    parser.add_argument(
        "--lower-slave", type=int, default=2, help="EtherCAT chain position of the elbow->wrist (forearm) drive."
    )
    parser.add_argument(
        "--lower-down-position", type=int, default=2000, help="Encoder position at the forearm 'down' calibration point."
    )
    parser.add_argument(
        "--lower-up-position",
        type=int,
        default=12000,
        help="Encoder position at the forearm 'up' calibration point.",
    )
    # The lower joint's angle convention is 0=pointing right, 90=up,
    # 180=pointing left (see JetsonPoseTracker._segment_angle_0_180), a
    # different scale than the upper joint's folded +/-90 convention above.
    parser.add_argument("--lower-down-angle-deg", type=float, default=0.0)
    parser.add_argument("--lower-up-angle-deg", type=float, default=180.0)
    parser.add_argument("--lower-velocity", type=int, default=5000)
    parser.add_argument("--lower-acceleration", type=int, default=5000)
    parser.add_argument("--lower-deceleration", type=int, default=5000)

    args = parser.parse_args()

    upper = _JointFollower(
        slave=args.upper_slave,
        down_position=args.upper_down_position,
        up_position=args.upper_up_position,
        down_angle_deg=args.upper_down_angle_deg,
        up_angle_deg=args.upper_up_angle_deg,
        velocity=args.upper_velocity,
        acceleration=args.upper_acceleration,
        deceleration=args.upper_deceleration,
        deadband_counts=args.deadband_counts,
        min_command_interval=args.min_command_interval,
        smoothing=args.smoothing,
    )
    lower = _JointFollower(
        slave=args.lower_slave,
        down_position=args.lower_down_position,
        up_position=args.lower_up_position,
        down_angle_deg=args.lower_down_angle_deg,
        up_angle_deg=args.lower_up_angle_deg,
        velocity=args.lower_velocity,
        acceleration=args.lower_acceleration,
        deceleration=args.lower_deceleration,
        deadband_counts=args.deadband_counts,
        min_command_interval=args.min_command_interval,
        smoothing=args.smoothing,
        debug=True,
    )

    controller = TwoJointArmController(
        upper=upper,
        lower=lower,
        interface=args.interface,
        camera_id=args.camera_id,
        arm_side=args.arm_side,
        preview=args.preview,
        frame_width=args.frame_width,
        frame_height=args.frame_height,
    )

    try:
        controller.start()
        # Also exit if the tracker thread stops on its own (e.g. the
        # preview window was closed with 'q', or the camera dropped out),
        # so shutdown always runs the same disable-both-drives sequence
        # instead of leaving the motor loop spinning with a dead camera.
        while controller.tracker.running:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
