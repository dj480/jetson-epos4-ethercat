import argparse
import time

from .jetson_pinch_service import JetsonHandTracker, PinchState


def main():
    parser = argparse.ArgumentParser(description="Headless pinch event client for JetsonHandTracker.")
    parser.add_argument("--camera-id", type=int, default=0, help="Camera index.")
    args = parser.parse_args()

    tracker = JetsonHandTracker(camera_id=args.camera_id)
    tracker.start()
    print("Started headless pinch client. Press Ctrl+C to stop.")

    try:
        last_state = None
        last_hold_time = 0.0
        while True:
            event = tracker.get_event(block=True, timeout=0.5)
            if event is None:
                continue

            state, x, y, hand = event
            now = time.monotonic()
            if state == PinchState.PINCH_DOWN:
                print(f"PINCH_DOWN  x={x:.3f} y={y:.3f} hand={hand}")
                last_state = state
                last_hold_time = now
            elif state == PinchState.PINCH_HOLD:
                # Only print at most once every 0.5s to reduce spam.
                if now - last_hold_time >= 0.5:
                    print(f"PINCH_HOLD  x={x:.3f} y={y:.3f} hand={hand}")
                    last_hold_time = now
                last_state = state
            elif state == PinchState.PINCH_UP:
                print(f"PINCH_UP    x={x:.3f} y={y:.3f} hand={hand}")
                last_state = state

    except KeyboardInterrupt:
        print("Stopping headless pinch client.")
    finally:
        tracker.stop()


if __name__ == "__main__":
    main()
