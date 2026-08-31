import argparse
import threading
import time

from .cli import motor
from .jetson_pose_service import JetsonPoseTracker


def _clamp(value, low, high):
    return max(low, min(high, value))


class ArmMotorController:
    """Drive the motor's absolute position from the shoulder-elbow angle.

    Unlike the pinch controller, which starts/stops a fixed continuous move,
    this maps a continuously changing angle onto a continuously changing
    absolute target position (slider-style). The vision callback only
    records the latest target; a separate motor thread issues the native
    calls so camera processing never blocks on EtherCAT I/O, mirroring
    PinchMotorController.
    """

    def __init__(
        self,
        interface="enP8p1s0",
        slave=1,
        camera_id=0,
        down_position=None,
        up_position=None,
        velocity=5000,
        acceleration=5000,
        deceleration=5000,
        arm_side="right",
        down_angle_deg=-30.0,
        up_angle_deg=60.0,
        deadband_counts=20,
        min_command_interval=0.1,
        smoothing=0.3,
        preview=False,
        frame_width=640,
        frame_height=480,
    ):
        if down_position is None or up_position is None:
            raise ValueError("down_position and up_position are required calibration points.")
        if down_position == up_position:
            raise ValueError("down_position and up_position must differ.")
        if up_angle_deg <= down_angle_deg:
            raise ValueError("up_angle_deg must be greater than down_angle_deg.")

        self.interface = interface
        self.slave = int(slave)
        self.camera_id = camera_id
        # These are the measured encoder counts at the down/up calibration
        # points, in whatever order they actually came out as - encoder
        # counts are not guaranteed to increase in the "up" direction, so
        # down_position can be larger than up_position. The safety clamp
        # range is derived from min()/max() of the two, independent of which
        # one means "down".
        self.down_position = int(down_position)
        self.up_position = int(up_position)
        self.min_position = min(self.down_position, self.up_position)
        self.max_position = max(self.down_position, self.up_position)
        self.velocity = velocity
        self.acceleration = int(acceleration)
        self.deceleration = int(deceleration)
        self.arm_side = arm_side
        self.down_angle_deg = float(down_angle_deg)
        self.up_angle_deg = float(up_angle_deg)
        self.deadband_counts = int(deadband_counts)
        self.min_command_interval = float(min_command_interval)
        self.smoothing = float(smoothing)
        self.preview = bool(preview)
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)

        self.tracker = None
        self.motor_thread = None
        self.running = False

        self.motor_enabled = False

        self._state_lock = threading.Lock()
        self._smoothed_angle = None
        self._target_position = None
        self._visible = False

        self._last_commanded_position = None
        self._last_command_time = 0.0

    def start(self):
        # Initialize the native EtherCAT layer first and leave the drive
        # disabled until a real pose measurement produces a target. This
        # prevents a camera startup or model-loading delay from causing
        # motor motion, matching PinchMotorController's start-up ordering.
        if motor.motor_init(self.interface.encode("utf-8")) <= 0:
            raise RuntimeError(f"Failed to initialize motor interface: {self.interface}")

        motor.motor_disable(self.slave)
        self.motor_enabled = False

        self.tracker = JetsonPoseTracker(
            camera_id=self.camera_id,
            arm_side=self.arm_side,
            on_pose_event=self._on_pose_event,
            enable_preview=self.preview,
            frame_width=self.frame_width,
            frame_height=self.frame_height,
        )
        self.tracker.start()

        self.running = True
        self.motor_thread = threading.Thread(target=self._motor_loop, daemon=True)
        self.motor_thread.start()

    def stop(self):
        # Stop the tracker and motor worker before unloading the shared C
        # library, same ordering as PinchMotorController.stop().
        self.running = False
        if self.tracker:
            try:
                self.tracker.stop()
            except Exception as e:
                print(f"[ArmMotorController] Error stopping tracker: {e}")

        if self.motor_thread and self.motor_thread.is_alive():
            try:
                self.motor_thread.join(timeout=3.0)
            except Exception as e:
                print(f"[ArmMotorController] Error joining motor thread: {e}")

        if self.motor_enabled:
            try:
                motor.motor_disable(self.slave)
                self.motor_enabled = False
            except Exception as e:
                print(f"[ArmMotorController] Error disabling motor: {e}")

        try:
            motor.motor_close()
        except Exception as e:
            print(f"[ArmMotorController] Error closing motor library: {e}")

    def _on_pose_event(self, angle_deg, visible):
        # Convert the vision service's angle into a target absolute position.
        # This callback deliberately does not call motor functions directly;
        # the motor thread turns the stored target into native calls.
        with self._state_lock:
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

            # Interpolate between the measured down/up positions directly
            # (not min/max) so the mapping is correct even if "up" turned
            # out to be the smaller encoder count.
            target = self.down_position + ratio * (self.up_position - self.down_position)
            self._target_position = int(round(target))
            self._visible = True

    def _motor_loop(self):
        # Keep motor I/O off the camera callback thread. Deadband and rate
        # limiting keep this from spamming SDO writes on frame-to-frame
        # jitter, since each absolute move already costs ~40ms of sleeps in
        # the native layer (c_src/motor.c's _issue_absolute_move).
        while self.running:
            with self._state_lock:
                visible = self._visible
                target = self._target_position

            if visible and target is not None:
                target = _clamp(target, self.min_position, self.max_position)
                now = time.monotonic()
                moved_enough = (
                    self._last_commanded_position is None
                    or abs(target - self._last_commanded_position) >= self.deadband_counts
                )
                waited_enough = (now - self._last_command_time) >= self.min_command_interval

                if moved_enough and waited_enough:
                    if not self.motor_enabled:
                        motor.motor_enable(self.slave)
                        motor.motor_set_velocity(self.slave, self.velocity)
                        motor.motor_set_acceleration(self.slave, self.acceleration, self.deceleration)
                        self.motor_enabled = True

                    motor.motor_move_absolute(self.slave, target)
                    self._last_commanded_position = target
                    self._last_command_time = now
            # else: pose lost or no measurement yet - hold last commanded
            # position rather than snapping anywhere.

            time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(
        description="Track the shoulder-elbow angle and drive the motor's absolute position to follow it."
    )
    parser.add_argument("--camera-id", type=int, default=0, help="Camera index for video capture.")
    parser.add_argument("--interface", type=str, default="enP8p1s0", help="EtherCAT interface name.")
    parser.add_argument(
        "--slave",
        type=int,
        default=1,
        help="1-based EtherCAT chain position of the drive to control (see bin/scan_slaves).",
    )
    parser.add_argument(
        "--down-position",
        type=int,
        required=True,
        help="Measured encoder position at the 'arm down' calibration point (python -m python.cli, then 'p').",
    )
    parser.add_argument(
        "--up-position",
        type=int,
        required=True,
        help="Measured encoder position at the 'arm up' calibration point (python -m python.cli, then 'p').",
    )
    parser.add_argument("--velocity", type=int, default=5000, help="Profile velocity for the motor.")
    parser.add_argument(
        "--acceleration",
        type=int,
        default=5000,
        help="Profile acceleration (ramp-up rate to --velocity). Lower = smoother, higher = snappier.",
    )
    parser.add_argument(
        "--deceleration",
        type=int,
        default=5000,
        help="Profile deceleration (ramp-down rate). Lower = smoother, higher = snappier.",
    )
    parser.add_argument("--arm-side", choices=["left", "right"], default="right", help="Which arm to track.")
    parser.add_argument(
        "--down-angle-deg",
        type=float,
        default=-30.0,
        help="Shoulder-elbow angle (degrees, 0=horizontal) that maps to --down-position.",
    )
    parser.add_argument(
        "--up-angle-deg",
        type=float,
        default=60.0,
        help="Shoulder-elbow angle (degrees, 0=horizontal) that maps to --up-position.",
    )
    parser.add_argument(
        "--deadband-counts",
        type=int,
        default=20,
        help="Minimum position change required before issuing a new move.",
    )
    parser.add_argument(
        "--min-command-interval",
        type=float,
        default=0.1,
        help="Minimum seconds between absolute-move commands.",
    )
    parser.add_argument(
        "--smoothing",
        type=float,
        default=0.3,
        help="Exponential moving average factor applied to the measured angle (0-1, lower = smoother).",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show a preview window with overlay for debugging pose detection.",
    )
    parser.add_argument("--frame-width", type=int, default=640, help="Preview frame width.")
    parser.add_argument("--frame-height", type=int, default=480, help="Preview frame height.")
    args = parser.parse_args()

    controller = ArmMotorController(
        interface=args.interface,
        slave=args.slave,
        camera_id=args.camera_id,
        down_position=args.down_position,
        up_position=args.up_position,
        velocity=args.velocity,
        acceleration=args.acceleration,
        deceleration=args.deceleration,
        arm_side=args.arm_side,
        down_angle_deg=args.down_angle_deg,
        up_angle_deg=args.up_angle_deg,
        deadband_counts=args.deadband_counts,
        min_command_interval=args.min_command_interval,
        smoothing=args.smoothing,
        preview=args.preview,
        frame_width=args.frame_width,
        frame_height=args.frame_height,
    )

    try:
        controller.start()
        print("Arm motor controller running. Press Ctrl+C to stop.")
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Stopping arm motor controller...")
    finally:
        controller.stop()
        print("Controller stopped.")


if __name__ == "__main__":
    main()
