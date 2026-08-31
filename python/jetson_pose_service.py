import math
import os
import threading
import queue
import time

import cv2

try:
    import mediapipe as mp
    HAS_MEDIAPIPE_TASKS = hasattr(mp, "tasks") and hasattr(mp.tasks, "vision")
    if HAS_MEDIAPIPE_TASKS:
        from mediapipe.tasks.python.vision.pose_landmarker import (
            PoseLandmarker,
            PoseLandmarkerOptions,
            PoseLandmark,
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
        PoseLandmarker = None
        PoseLandmarkerOptions = None
        PoseLandmark = None
        MpImage = None
        VisionTaskRunningMode = None
        BaseOptions = None
except Exception as exc:
    mp = None
    HAS_MEDIAPIPE_TASKS = False
    PoseLandmarker = None
    PoseLandmarkerOptions = None
    PoseLandmark = None
    MpImage = None
    VisionTaskRunningMode = None
    BaseOptions = None
    print(f"[JetsonPoseTracker] MediaPipe import failed: {exc}")


class JetsonPoseTracker:
    """Capture frames and publish arm-segment angles.

    The tracked joints in the physical rig are fixed pivots that only
    rotate, so the useful signal is each segment vector's angle, not the
    joint's raw position. Angle is measured from horizontal: 0 when the
    segment is level, positive as its far end rises, negative as it drops.
    There is no OpenCV fallback here (unlike the hand tracker) because pose
    has no simple contour approximation.

    By default only the shoulder->elbow segment is tracked, dispatched
    through ``on_pose_event``/``get_event`` as ``(angle_deg, visible)`` -
    this is the original single-segment behavior, unchanged so existing
    consumers (e.g. ArmMotorController) keep working as-is: 0 when level,
    +/-90 at fully up/down, folded so either horizontal direction reads 0
    (see ``_segment_angle``). Passing ``track_wrist=True`` additionally
    tracks the elbow->wrist segment's own absolute angle from the same
    detection pass (one camera, one MediaPipe inference per frame) and
    dispatches it through ``on_lower_pose_event`` as ``(lower_angle_deg,
    lower_visible)``, using a different, unfolded 0-180 convention: 0
    pointing right, 90 up, 180 pointing left (see ``_segment_angle_0_180``)
    - this fits a joint that sweeps through a full right->up->left
    half-turn rather than folding symmetrically around vertical. Both
    angles are absolute (from horizontal), so rotating the upper arm alone
    also moves the lower angle, since the forearm swings with it - this is
    intentional for this rig rather than an elbow-relative angle. The two
    segments have independent visibility, since the wrist can be occluded
    while the elbow is not.

    Detection runs on a daemon thread, matching JetsonHandTracker, throttled
    at ``frame_interval``. ``visible`` is False whenever a segment's
    landmarks are not confidently detected in the current frame, so a
    consumer can decide to hold its last commanded state instead of reacting
    to a missing measurement.
    """

    def __init__(
        self,
        camera_id=0,
        frame_width=160,
        frame_height=120,
        frame_interval=0.04,
        arm_side="right",
        on_pose_event=None,
        track_wrist=False,
        on_lower_pose_event=None,
        model_path=None,
        enable_preview=False,
        min_visibility=0.5,
    ):
        self.camera_id = camera_id
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.frame_interval = frame_interval
        self.arm_side = arm_side.lower()
        self.on_pose_event = on_pose_event
        self.track_wrist = bool(track_wrist)
        self.on_lower_pose_event = on_lower_pose_event
        self.enable_preview = bool(enable_preview)
        self.min_visibility = min_visibility

        if self.arm_side not in ("left", "right"):
            raise ValueError(f"arm_side must be 'left' or 'right', got {arm_side!r}")

        # Resolve the model from the repository rather than the process's
        # current directory, same rationale as JetsonHandTracker.
        if model_path is None:
            project_root = os.path.dirname(os.path.dirname(__file__))
            model_path = os.path.join(project_root, "models", "pose_landmarker_lite.task")

        self.model_path = os.fspath(model_path) if model_path is not None else None

        self.event_queue = queue.Queue(maxsize=50)
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

        self.latest_frame = None
        self.pose_landmarker = None

        if not HAS_MEDIAPIPE_TASKS or not self.model_path or not os.path.isfile(self.model_path):
            raise RuntimeError(
                "MediaPipe Pose Landmarker model not available; "
                f"expected a file at {self.model_path}"
            )

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
        self._close_pose_landmarker()

    def _close_pose_landmarker(self):
        if self.pose_landmarker is not None:
            try:
                self.pose_landmarker.close()
            except Exception:
                pass
            self.pose_landmarker = None

    def get_latest_frame(self):
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def get_event(self, block=False, timeout=None):
        try:
            return self.event_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    def _dispatch_event(self, angle_deg, visible):
        event_data = (angle_deg, visible)
        if not self.event_queue.full():
            self.event_queue.put(event_data)
        if self.on_pose_event:
            try:
                self.on_pose_event(angle_deg, visible)
            except Exception as e:
                print(f"[JetsonPoseTracker] Callback error: {e}")

    def _dispatch_lower_event(self, lower_angle_deg, lower_visible):
        if self.on_lower_pose_event:
            try:
                self.on_lower_pose_event(lower_angle_deg, lower_visible)
            except Exception as e:
                print(f"[JetsonPoseTracker] Lower callback error: {e}")

    def _create_pose_landmarker(self):
        if self.pose_landmarker is not None:
            return

        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.model_path),
            running_mode=VisionTaskRunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.pose_landmarker = PoseLandmarker.create_from_options(options)

    def _is_visible(self, landmark):
        vis = getattr(landmark, "visibility", 1.0)
        return vis is None or vis >= self.min_visibility

    @staticmethod
    def _segment_angle(near_pt, far_pt):
        # Angle of the near->far vector, measured from horizontal, "up"
        # positive. Image y increases downward, so negate dy. abs(dx) keeps
        # the range at +/-90 degrees regardless of which side the far point
        # points toward.
        dx = far_pt[0] - near_pt[0]
        dy = far_pt[1] - near_pt[1]
        return math.degrees(math.atan2(-dy, abs(dx)))

    @staticmethod
    def _segment_angle_0_180(near_pt, far_pt):
        # Angle of the near->far vector over a continuous 0-180 degree
        # sweep: 0 pointing right, 90 pointing up, 180 pointing left. Unlike
        # _segment_angle, dx is signed (not abs()'d), so right and left are
        # distinct instead of mirrored - correct for a joint that swings
        # through a full right->up->left half-turn rather than folding
        # symmetrically around vertical.
        #
        # atan2's own range is (-180, 180], so a vector pointing left that
        # dips even slightly below horizontal (dx<0, dy>0) comes back as a
        # large negative angle near -180 - mathematically the same
        # direction as +180, but with no memory that it arrived from the
        # "left/up" side rather than the "right/down" side, so left at
        # face value it reads as if pointing almost straight right. -90
        # (straight down) is the natural dividing line between "dipped
        # below on the right" (small negative, e.g. -10 - already handled
        # correctly by the down_angle_deg clamp elsewhere) and "dipped
        # below on the left" (large negative, e.g. -170): unwrapping only
        # the second case by +360 keeps this joint continuous through 180
        # instead of snapping to the opposite end.
        dx = far_pt[0] - near_pt[0]
        dy = far_pt[1] - near_pt[1]
        angle = math.degrees(math.atan2(-dy, dx))
        if angle < -90:
            angle += 360
        return angle

    def _find_pose_angles(self, frame):
        """Run one detection pass and extract the shoulder->elbow angle,
        plus the elbow->wrist angle when track_wrist is set. Returns None
        only if no pose at all was detected; each segment's visibility is
        otherwise tracked independently in the returned dict.

        Both angles are absolute, measured from horizontal - the lower
        angle is the forearm's own orientation, not relative to the upper
        arm. This means rotating the upper arm alone also moves the lower
        angle (the forearm swings with it), which is intentional per user
        preference for this rig rather than an elbow-relative angle."""
        self._create_pose_landmarker()
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = MpImage(ImageFormat.SRGB, rgb_frame)
        timestamp_ms = int(time.time() * 1000)
        results = self.pose_landmarker.detect_for_video(mp_image, timestamp_ms)
        if not results.pose_landmarks:
            return None

        landmarks = results.pose_landmarks[0]
        if self.arm_side == "left":
            shoulder = landmarks[PoseLandmark.LEFT_SHOULDER]
            elbow = landmarks[PoseLandmark.LEFT_ELBOW]
            wrist = landmarks[PoseLandmark.LEFT_WRIST]
        else:
            shoulder = landmarks[PoseLandmark.RIGHT_SHOULDER]
            elbow = landmarks[PoseLandmark.RIGHT_ELBOW]
            wrist = landmarks[PoseLandmark.RIGHT_WRIST]

        h, w = frame.shape[:2]
        shoulder_pt = (shoulder.x * w, shoulder.y * h)
        elbow_pt = (elbow.x * w, elbow.y * h)
        wrist_pt = (wrist.x * w, wrist.y * h)

        result = {"upper_visible": False, "lower_visible": False}

        if self._is_visible(shoulder) and self._is_visible(elbow):
            result["angle_deg"] = self._segment_angle(shoulder_pt, elbow_pt)
            result["shoulder_pt"] = (int(shoulder_pt[0]), int(shoulder_pt[1]))
            result["elbow_pt"] = (int(elbow_pt[0]), int(elbow_pt[1]))
            result["upper_visible"] = True

        if self.track_wrist and self._is_visible(elbow) and self._is_visible(wrist):
            result["lower_angle_deg"] = self._segment_angle_0_180(elbow_pt, wrist_pt)
            result["elbow_pt"] = (int(elbow_pt[0]), int(elbow_pt[1]))
            result["wrist_pt"] = (int(wrist_pt[0]), int(wrist_pt[1]))
            result["lower_visible"] = True

        return result

    def _update_loop(self):
        cap = cv2.VideoCapture(self.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            print(f"[JetsonPoseTracker] Failed to open camera at index {self.camera_id}. Retrying index 1...")
            cap = cv2.VideoCapture(1)

        segments = "shoulder->elbow->wrist" if self.track_wrist else "shoulder->elbow"
        print(f"[JetsonPoseTracker] Tracking {self.arm_side} {segments} angle(s).")

        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                time.sleep(self.frame_interval)
                continue

            if self.enable_preview:
                frame = cv2.flip(frame, 1)

            pose_result = self._find_pose_angles(frame)
            if pose_result is not None:
                self._dispatch_event(pose_result.get("angle_deg", 0.0), pose_result["upper_visible"])
                if self.track_wrist:
                    self._dispatch_lower_event(pose_result.get("lower_angle_deg", 0.0), pose_result["lower_visible"])

                if self.enable_preview:
                    if pose_result["upper_visible"]:
                        cv2.circle(frame, pose_result["shoulder_pt"], 8, (255, 0, 0), -1)
                        cv2.circle(frame, pose_result["elbow_pt"], 8, (0, 255, 255), -1)
                        cv2.line(frame, pose_result["shoulder_pt"], pose_result["elbow_pt"], (0, 255, 0), 2)
                        cv2.putText(
                            frame,
                            f"upper={pose_result['angle_deg']:.1f} deg  side={self.arm_side}",
                            (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                        )
                    if self.track_wrist and pose_result["lower_visible"]:
                        cv2.circle(frame, pose_result["wrist_pt"], 8, (255, 0, 255), -1)
                        cv2.line(frame, pose_result["elbow_pt"], pose_result["wrist_pt"], (0, 128, 255), 2)
                        cv2.putText(
                            frame,
                            f"lower={pose_result['lower_angle_deg']:.1f} deg",
                            (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 128, 255),
                            2,
                        )
            else:
                self._dispatch_event(0.0, False)
                if self.track_wrist:
                    self._dispatch_lower_event(0.0, False)

            if self.enable_preview:
                if pose_result is None:
                    cv2.putText(
                        frame,
                        "No pose detected",
                        (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2,
                    )
                try:
                    cv2.imshow("JetsonPoseTracker", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.running = False
                        break
                except Exception:
                    pass

            if self.enable_preview:
                with self.lock:
                    self.latest_frame = frame

            if self.frame_interval > 0:
                time.sleep(self.frame_interval)

        cap.release()
        try:
            if self.enable_preview:
                cv2.destroyWindow("JetsonPoseTracker")
        except Exception:
            pass
