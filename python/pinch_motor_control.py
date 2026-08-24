import argparse
import threading
import time

from .cli import motor
from .jetson_pinch_service import JetsonHandTracker, PinchState


class PinchMotorController:
    """Translate pinch lifecycle events into safe motor operations.

    The tracker callback only records gesture state. A separate motor thread
    performs the native calls because camera processing must not block on
    EtherCAT I/O. While a pinch is active, the drive is enabled and one
    continuous move is started; release stops that move. Positive counts are
    used for the left-hand gesture and negative counts for the right-hand
    gesture.
    """

    def __init__(
        self,
        interface="enP8p1s0",
        camera_id=0,
        velocity=5000,
        step_counts=250,
        use_fallback=False,
        preview=False,
        frame_width=160,
        frame_height=120,
    ):
        self.interface = interface
        self.camera_id = camera_id
        self.velocity = velocity
        self.step_counts = step_counts
        self.use_fallback = use_fallback
        self.preview = bool(preview)
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)

        self.tracker = None
        self.motor_thread = None
        self.running = False
        self.pinch_active = False

        self.motor_enabled = False
        self.motor_move_in_progress = False
        self.should_disable_after_move = False
        self._continuous_running = False
        self._current_step_counts = int(self.step_counts)

    def start(self):
        # Initialize the native EtherCAT layer first and leave the drive
        # disabled until a real PINCH_DOWN event arrives. This prevents a
        # camera startup or model-loading delay from causing motor motion.
        if motor.motor_init(self.interface.encode("utf-8")) <= 0:
            raise RuntimeError(f"Failed to initialize motor interface: {self.interface}")

        motor.motor_disable()
        self.motor_enabled = False

        self.tracker = JetsonHandTracker(
            camera_id=self.camera_id,
            on_pinch_event=self._on_pinch_event,
            use_fallback=self.use_fallback,
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
        # library. Both workers can still issue native calls, so closing the
        # library first would create a race during shutdown.
        self.running = False
        if self.tracker:
            try:
                self.tracker.stop()
            except Exception as e:
                print(f"[PinchMotorController] Error stopping tracker: {e}")

        if self.motor_thread and self.motor_thread.is_alive():
            try:
                self.motor_thread.join(timeout=3.0)
            except Exception as e:
                print(f"[PinchMotorController] Error joining motor thread: {e}")

        if self.motor_enabled:
            try:
                self._disable_motor()
            except Exception as e:
                print(f"[PinchMotorController] Error disabling motor: {e}")

        try:
            motor.motor_close()
        except Exception as e:
            print(f"[PinchMotorController] Error closing motor library: {e}")

    def _on_pinch_event(self, state, x, y, hand):
        # Convert the vision service's hand label into a signed encoder move.
        # The callback deliberately does not call motor functions directly.
        if hand is None:
            hand = "left"

        # DOWN starts the command, HOLD keeps the intent alive, and UP removes
        # it. The motor loop turns that intent into idempotent start/stop calls.
        if state == PinchState.PINCH_DOWN:
            print(f"[PINCH] PINCH_DOWN -> continuous motion started (hand={hand})")
            self.pinch_active = True
            # store current sign-mapped counts for continuous motion
            self._current_step_counts = int(self.step_counts if hand == "left" else -abs(self.step_counts))
        elif state == PinchState.PINCH_HOLD:
            self.pinch_active = True
        elif state == PinchState.PINCH_UP:
            print(f"[PINCH] PINCH_UP -> stopping continuous motion (hand={hand})")
            self.pinch_active = False

    def _motor_loop(self):
        # Keep motor I/O off the camera callback thread. The loop also avoids
        # repeatedly starting the same continuous command for every HOLD
        # event, which is important because HOLD is emitted every frame.
        while self.running:
            if self.pinch_active:
                if not self.motor_enabled:
                    motor.motor_enable()
                    self.motor_enabled = True
                if not self._continuous_running:
                    motor.motor_set_velocity(self.velocity)
                    motor.motor_start_continuous(self._current_step_counts, 5)
                    self._continuous_running = True
            else:
                if self._continuous_running:
                    motor.motor_stop_continuous()
                    self._continuous_running = False

            time.sleep(0.02)

    def _perform_move(self, counts, velocity):
        # Guard the retained one-shot movement path against overlapping moves.
        if self.motor_move_in_progress:
            return

        if not self.motor_enabled:
            motor.motor_enable()
            self.motor_enabled = True

        motor.motor_set_velocity(velocity)
        self.motor_move_in_progress = True
        motor.motor_move_relative(counts)
        self.motor_move_in_progress = False

        if self.should_disable_after_move:
            self._disable_motor()
            self.should_disable_after_move = False

    def _request_disable(self):
        # Defer disabling until an in-flight command has completed.
        if self.motor_move_in_progress:
            self.should_disable_after_move = True
        else:
            self._disable_motor()

    def _disable_motor(self):
        motor.motor_disable()
        self.motor_enabled = False


def main():
    # Expose hardware, camera, and detection settings as command-line options.
    parser = argparse.ArgumentParser(description="Headless pinch-to-motor controller.")
    parser.add_argument("--camera-id", type=int, default=0, help="Camera index for video capture.")
    parser.add_argument("--interface", type=str, default="enP8p1s0", help="EtherCAT interface name.")
    parser.add_argument("--velocity", type=int, default=5000, help="Profile velocity for the motor.")
    parser.add_argument("--step-counts", type=int, default=250, help="Relative counts to move on pinch down.")
    parser.add_argument(
        "--use-fallback",
        action="store_true",
        help="Force OpenCV fallback for pinch detection instead of MediaPipe.",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show a preview window with overlay for debugging hand detection.",
    )
    parser.add_argument("--frame-width", type=int, default=160, help="Preview frame width.")
    parser.add_argument("--frame-height", type=int, default=120, help="Preview frame height.")
    args = parser.parse_args()

    controller = PinchMotorController(
        interface=args.interface,
        camera_id=args.camera_id,
        velocity=args.velocity,
        step_counts=args.step_counts,
        use_fallback=args.use_fallback,
        preview=args.preview,
        frame_width=args.frame_width,
        frame_height=args.frame_height,
    )

    try:
        controller.start()
        print("Pinch motor controller running. Press Ctrl+C to stop.")
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Stopping pinch motor controller...")
    finally:
        controller.stop()
        print("Controller stopped.")


if __name__ == "__main__":
    main()
