import cv2
import math
import numpy as np
import os
import threading
import queue
import time
from enum import Enum, auto

try:
    import mediapipe as mp
    HAS_MEDIAPIPE_TASKS = hasattr(mp, "tasks") and hasattr(mp.tasks, "vision")
    if HAS_MEDIAPIPE_TASKS:
        from mediapipe.tasks.python.vision.hand_landmarker import (
            HandLandmarker,
            HandLandmarkerOptions,
        )
        from mediapipe.tasks.python.vision.core.image import (
            Image as MpImage,
            ImageFormat,
        )
        from mediapipe.tasks.python.vision.core.vision_task_running_mode import (
            VisionTaskRunningMode,
        )
        from mediapipe.tasks.python import BaseOptions
    else:
        HandLandmarker = None
        HandLandmarkerOptions = None
        MpImage = None
        VisionTaskRunningMode = None
        BaseOptions = None
except Exception as exc:
    mp = None
    HAS_MEDIAPIPE_TASKS = False
    HandLandmarker = None
    HandLandmarkerOptions = None
    MpImage = None
    VisionTaskRunningMode = None
    BaseOptions = None
    print(f"[JetsonHandTracker] MediaPipe import failed: {exc}")

class PinchState(Enum):
    OPEN = auto()
    PINCH_DOWN = auto()
    PINCH_HOLD = auto()
    PINCH_UP = auto()

class JetsonHandTracker:
    """Headless pinch tracker optimized for Jetson with MediaPipe support."""

    def __init__(
        self,
        camera_id=0,
        frame_width=160,
        frame_height=120,
        frame_interval=0.04,
        close_threshold=0.24,
        open_threshold=0.32,
        on_pinch_event=None,
        model_path="hand_landmarker.task",
        use_fallback=None,
        enable_preview=False,
        min_contour_area=1200,
    ):
        self.camera_id = camera_id
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.frame_interval = frame_interval
        self.close_threshold = close_threshold
        self.open_threshold = open_threshold
        self.on_pinch_event = on_pinch_event
        self.enable_preview = bool(enable_preview)
        self.min_contour_area = min_contour_area
        self.model_path = os.fspath(model_path) if model_path is not None else None

        self.event_queue = queue.Queue(maxsize=50)
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

        self.latest_frame = None
        self.is_currently_pinching = False
        self.hand_landmarker = None

        if use_fallback is None:
            self.use_opencv_fallback = not (
                HAS_MEDIAPIPE_TASKS
                and self.model_path
                and os.path.isfile(self.model_path)
            )
        else:
            self.use_opencv_fallback = bool(use_fallback)

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self._close_hand_landmarker()

    def _close_hand_landmarker(self):
        if self.hand_landmarker is not None:
            try:
                self.hand_landmarker.close()
            except Exception:
                pass
            self.hand_landmarker = None

    def get_latest_frame(self):
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def get_event(self, block=False, timeout=None):
        try:
            return self.event_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    def _dispatch_event(self, state, norm_x, norm_y):
        event_data = (state, norm_x, norm_y)
        if not self.event_queue.full():
            self.event_queue.put(event_data)
        if self.on_pinch_event:
            try:
                self.on_pinch_event(state, norm_x, norm_y)
            except Exception as e:
                print(f"[JetsonHandTracker] Callback error: {e}")

    def _create_skin_mask(self, frame):
        ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        ycrcb_mask = cv2.inRange(ycrcb, (0, 133, 77), (255, 173, 127))
        hsv_mask = cv2.inRange(hsv, (0, 20, 70), (25, 255, 255))

        mask = cv2.bitwise_or(ycrcb_mask, hsv_mask)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    def _create_hand_landmarker(self):
        if self.hand_landmarker is not None:
            return
        if not HAS_MEDIAPIPE_TASKS or not self.model_path or not os.path.isfile(self.model_path):
            raise RuntimeError("MediaPipe model not available for hand landmarker.")

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.model_path),
            running_mode=VisionTaskRunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.hand_landmarker = HandLandmarker.create_from_options(options)

    def _find_pinch(self, frame):
        if self.use_opencv_fallback:
            return self._opencv_pinch_detection(frame)

        self._create_hand_landmarker()
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = MpImage(ImageFormat.SRGB, rgb_frame)
        timestamp_ms = int(time.time() * 1000)
        results = self.hand_landmarker.detect_for_video(mp_image, timestamp_ms)
        if not results.hand_landmarks:
            return None

        hand_landmarks = results.hand_landmarks[0]
        h, w = frame.shape[:2]
        raw_pts = np.array([[lm.x * w, lm.y * h] for lm in hand_landmarks], dtype=np.float32)

        thumb_tip = raw_pts[4]
        index_tip = raw_pts[8]
        wrist = raw_pts[0]
        middle_mcp = raw_pts[9]

        palm_scale = np.linalg.norm(wrist - middle_mcp)
        if palm_scale < 1e-5:
            palm_scale = 1.0

        pinch_dist = np.linalg.norm(thumb_tip - index_tip)
        norm_pinch_dist = float(pinch_dist / palm_scale)
        midpoint = ((thumb_tip[0] + index_tip[0]) / 2.0, (thumb_tip[1] + index_tip[1]) / 2.0)
        norm_x = float(midpoint[0] / w)
        norm_y = float(midpoint[1] / h)

        return {
            "norm_pinch_dist": norm_pinch_dist,
            "norm_x": norm_x,
            "norm_y": norm_y,
            "p0": (int(thumb_tip[0]), int(thumb_tip[1])),
            "p1": (int(index_tip[0]), int(index_tip[1])),
        }

    def _opencv_pinch_detection(self, frame):
        mask = self._create_skin_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < self.min_contour_area:
            return None

        hull = cv2.convexHull(contour, returnPoints=True)
        if hull.shape[0] < 5:
            return None

        centroid = self._get_contour_centroid(contour)
        if centroid is None:
            return None

        top_pts = [tuple(pt[0]) for pt in hull if pt[0][1] < centroid[1] + 30]
        if len(top_pts) < 2:
            top_pts = [tuple(pt[0]) for pt in hull]

        best_pair = None
        best_dist = float("inf")
        for i in range(len(top_pts)):
            for j in range(i + 1, len(top_pts)):
                dx = top_pts[i][0] - top_pts[j][0]
                dy = top_pts[i][1] - top_pts[j][1]
                dist = math.hypot(dx, dy)
                if dist < best_dist:
                    best_dist = dist
                    best_pair = (top_pts[i], top_pts[j])

        if best_pair is None:
            return None

        p0, p1 = best_pair
        bbox = cv2.boundingRect(contour)
        palm_scale = max(bbox[3], 1)
        norm_pinch_dist = best_dist / palm_scale
        midpoint = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
        h, w = frame.shape[:2]
        norm_x = float(midpoint[0] / w)
        norm_y = float(midpoint[1] / h)

        return {
            "norm_pinch_dist": norm_pinch_dist,
            "norm_x": norm_x,
            "norm_y": norm_y,
            "p0": (int(p0[0]), int(p0[1])),
            "p1": (int(p1[0]), int(p1[1])),
            "contour": contour,
        }

    def _get_contour_centroid(self, contour):
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None
        return (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])

    def _update_loop(self):
        cap = cv2.VideoCapture(self.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            print(f"[JetsonHandTracker] Failed to open camera at index {self.camera_id}. Retrying index 1...")
            cap = cv2.VideoCapture(1)

        if self.use_opencv_fallback:
            print("[JetsonHandTracker] MediaPipe unavailable; using OpenCV fallback.")
        else:
            print(
                "[JetsonHandTracker] Using MediaPipe hand landmarker with low-resolution, low-frame-rate mode."
            )

        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                time.sleep(self.frame_interval)
                continue

            if self.enable_preview:
                frame = cv2.flip(frame, 1)

            pinch_result = self._find_pinch(frame)
            if pinch_result is not None:
                norm_pinch_dist = pinch_result["norm_pinch_dist"]
                norm_x = pinch_result["norm_x"]
                norm_y = pinch_result["norm_y"]

                if not self.is_currently_pinching:
                    if norm_pinch_dist <= self.close_threshold:
                        self.is_currently_pinching = True
                        self._dispatch_event(PinchState.PINCH_DOWN, norm_x, norm_y)
                else:
                    if norm_pinch_dist >= self.open_threshold:
                        self.is_currently_pinching = False
                        self._dispatch_event(PinchState.PINCH_UP, norm_x, norm_y)
                    else:
                        self._dispatch_event(PinchState.PINCH_HOLD, norm_x, norm_y)

                if self.enable_preview:
                    color = (0, 255, 0) if self.is_currently_pinching else (0, 0, 255)
                    state_label = "PINCH" if self.is_currently_pinching else "OPEN"
                    if "contour" in pinch_result:
                        cv2.drawContours(frame, [pinch_result["contour"]], -1, (255, 255, 0), 2)
                    cv2.circle(frame, pinch_result["p0"], 8, (255, 0, 0), -1)
                    cv2.circle(frame, pinch_result["p1"], 8, (0, 255, 255), -1)
                    cv2.putText(
                        frame,
                        f"State: {state_label} ({norm_pinch_dist:.2f})",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )
            else:
                if self.is_currently_pinching:
                    self.is_currently_pinching = False
                    self._dispatch_event(PinchState.PINCH_UP, 0.0, 0.0)

            if self.enable_preview:
                with self.lock:
                    self.latest_frame = frame

            if self.frame_interval > 0:
                time.sleep(self.frame_interval)

        cap.release()

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run JetsonHandTracker headless or with preview.")
    parser.add_argument("--camera-id", type=int, default=0, help="Camera index for cv2.VideoCapture.")
    parser.add_argument("--preview", action="store_true", help="Show preview window.")
    parser.add_argument("--frame-interval", type=float, default=0.12, help="Seconds between capture frames.")
    parser.add_argument("--min-contour-area", type=int, default=1200, help="Minimum contour size for pinch detection.")
    parser.add_argument("--use-fallback", action="store_true", help="Force OpenCV fallback instead of MediaPipe.")
    args = parser.parse_args()

    def sample_callback(state, x, y):
        print(f"[EVENT] State: {state.name:<10} | Norm Coords: X={x:.3f}, Y={y:.3f}")

    tracker = JetsonHandTracker(
        camera_id=args.camera_id,
        frame_width=240,
        frame_height=160,
        frame_interval=args.frame_interval,
        min_contour_area=args.min_contour_area,
        on_pinch_event=sample_callback,
        use_fallback=args.use_fallback,
        enable_preview=args.preview,
    )
    tracker.start()

    try:
        if args.preview:
            print("Preview enabled. Press 'q' to stop.")
            while True:
                frame = tracker.get_latest_frame()
                if frame is not None:
                    cv2.imshow("Jetson Hand Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        else:
            print("Headless tracker running. Press Ctrl+C to stop.")
            while True:
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop()
        if args.preview:
            cv2.destroyAllWindows()
        print("Tracker stopped.")
