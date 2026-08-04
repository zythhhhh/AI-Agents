# LEROBOT TRAINING POLICY
# used YOLO + IK to create pick and place movements, then used those movements to train a policy

# general imports
from __future__ import annotations
import time
import numpy as np
import cv2
import threading
import logging
from pathlib import Path
from enum import Enum, auto
from typing import Any

# YOLO + IKPY imports
from ultralytics import YOLOWorld
from ikpy.chain import Chain
from ikpy.urdf import URDF


# lerobot imports
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.processor import make_default_processors
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.feature_utils import hw_to_dataset_features
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_visualization

# Keyboard listener moved between LeRobot versions.
try:
    from lerobot.utils.keyboard_input import init_keyboard_listener
except ImportError:
    try:
        from lerobot.common.control_utils import init_keyboard_listener
    except ImportError:
        from pynput import keyboard

        def init_keyboard_listener():
            events = {
                "exit_early": False,
                "rerecord_episode": False,
                "stop_recording": False,
            }

            def on_press(key):
                try:
                    char = getattr(key, "char", None)

                    if key == keyboard.Key.right or char == "n":
                        print("Next episode requested.")
                        events["exit_early"] = True

                    elif key == keyboard.Key.left or char == "r":
                        print("Re-recording current episode.")
                        events["rerecord_episode"] = True
                        events["exit_early"] = True

                    elif key == keyboard.Key.esc or char == "q":
                        print("Stopping recording session.")
                        events["stop_recording"] = True
                        events["exit_early"] = True

                except Exception as exc:
                    print(f"Keyboard listener error: {exc}")

            listener = keyboard.Listener(on_press=on_press)
            listener.start()
            return listener, events


# User configs
PORT = "COM8"
ROBOT_ID = "like_and_follow_er_arm"
CAMERA_INDEX = 0

REPO_ID = "Schisttherock/Manufacturing-Task-SO101"
TASK_DESCRIPTION = "Grab the cube highest up and place it on the conveyor belt below"

NUM_EPISODES = 5
FPS = 30
EPISODE_TIME_SEC = 60
RESET_TIME_SEC = 10

URDF_PATH = Path(__file__).parent / "SO101" / "so101_new_calib.urdf"

# constants
HOVER_HEIGHT = 0.10
CUBE_HEIGHT = -0.005
DROP_HEIGHT = -0.11
MOVE_DURATION = 1.5

# Safety limits
MAX_IK_ERROR_M = 0.015       # 15 mm
MIN_DETECTION_CONF = 0.20
STABLE_SAMPLE_COUNT = 10

# Gripper values
GRIPPER_OPEN = 35.0
GRIPPER_CLOSED = 10.0

ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

# wrist_roll is held fixed during IK.
IK_ACTIVE_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
]

# Camera-to-robot planar homography.
PIXEL_POINTS = np.array(
    [
        [576, 300],
        [80, 310],
        [555, 143],
        [61, 156],
        [320, 220],
    ],
    dtype=np.float32,
)

ROBOT_POINTS = np.array(
    [
        [0.2640, 0.2573],
        [0.2593, 0.1072],
        [0.2161, 0.2401],
        [0.2137, 0.0928],
        [0.2409, 0.1812],
    ],
    dtype=np.float32,
)

# Convert torch-like tensors or arrays to NumPy
def to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)

# Normalize a LeRobot image to HWC RGB uint8.
def image_to_rgb_uint8(image: Any) -> np.ndarray:
    array = to_numpy(image)
    if array.ndim != 3:
        raise ValueError(f"Expected a 3D image, got shape {array.shape}")
    # CHW -> HWC
    if array.shape[0] in (1, 3, 4) and array.shape[-1] not in (1, 3, 4):
        array = np.transpose(array, (1, 2, 0))

    if array.shape[-1] == 1:
        array = np.repeat(array, 3, axis=-1)
    elif array.shape[-1] == 4:
        array = array[..., :3]

    if np.issubdtype(array.dtype, np.floating):
        if array.size and float(np.nanmax(array)) <= 1.0:
            array = array * 255.0
        array = np.clip(array, 0, 255).astype(np.uint8)
    else:
        array = np.clip(array, 0, 255).astype(np.uint8)

    return np.ascontiguousarray(array)

# Define states for robot pick-and-place action
class RobotState(Enum):
    SEARCH = auto()
    MOVE_ABOVE = auto()
    LOWER = auto()
    GRASP = auto()
    LIFT = auto()
    MOVE_TO_BIN = auto()
    OPEN = auto()
    RETURN = auto()


class YOLOExpertTeleop(Teleoperator):
    """
    Algorithmic LeRobot teleoperator.
    Inherits from Teleoperator (subclass), but does not call Teleoperator .__init__
    because this virtual teleoperator has no motors or calibration file of its own.
    """

    name = "yolo_script"
    config_class = object

    def __init__(self, follower_robot: SO101Follower):
        self.id = "yolo_script"
        self.robot = follower_robot

        self._connected = False
        self._enabled = True

        self.state = RobotState.SEARCH
        self.trajectory: list[dict[str, Any]] = []

        self.pixel_samples: list[list[float]] = []
        self.sampled_label: str | None = None

        self.target_xyz: np.ndarray | None = None
        self.target_color: str | None = None

        self.gripper_pos = GRIPPER_OPEN
        self.current_joints = np.zeros(len(ARM_JOINTS), dtype=float)

        # YOLO
        self.model = YOLOWorld("yolov8s-worldv2.pt")
        self.model.set_classes(
            ["red cube", "blue cube", "yellow cube", "green cube"]
        )

        # Homography
        self.H, _ = cv2.findHomography(
            PIXEL_POINTS,
            ROBOT_POINTS,
            method=cv2.RANSAC,
        )
        if self.H is None:
            raise RuntimeError("Could not calculate camera-to-robot homography.")

        # IK
        if not URDF_PATH.exists():
            raise FileNotFoundError(f"URDF not found:\n{URDF_PATH}")

        joint_path = ARM_JOINTS + ["gripper_frame_joint"]
        base_elements = URDF.get_chain_from_joints(
            str(URDF_PATH),
            joint_path,
        )

        temp_chain = Chain.from_urdf_file(
            str(URDF_PATH),
            base_elements=base_elements,
        )

        active_mask = [
            link.name in IK_ACTIVE_JOINTS
            for link in temp_chain.links
        ]

        self.chain = Chain.from_urdf_file(
            str(URDF_PATH),
            base_elements=base_elements,
            active_links_mask=active_mask,
        )

        self.link_indices = {
            link.name: index
            for index, link in enumerate(self.chain.links)
        }

        missing = [
            joint for joint in ARM_JOINTS
            if joint not in self.link_indices
        ]
        if missing:
            raise RuntimeError(
                f"These joints were not found in the IK chain: {missing}"
            )

        print("\nIK chain:")
        for index, link in enumerate(self.chain.links):
            status = "ACTIVE" if self.chain.active_links_mask[index] else "fixed"
            print(f"  {index}: {link.name:25s} {status}")

        # Inference thread
        self.running = False
        self.lock = threading.Lock()
        self.latest_rgb: np.ndarray | None = None
        self.latest_detections: list[dict[str, Any]] = []
        self.inference_thread: threading.Thread | None = None

    # Required Teleoperator interface
    @property
    def action_features(self) -> dict[str, type]:
        return self.robot.action_features

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return

        log_say("YOLO algorithm initialized")
        self.running = True
        self._connected = True

        self.inference_thread = threading.Thread(
            target=self._inference_worker,
            daemon=True,
            name="yolo-inference",
        )
        self.inference_thread.start()

    def disconnect(self) -> None:
        if not self._connected:
            return

        log_say("YOLO algorithm shutting down")
        self.running = False
        self._connected = False

        if self.inference_thread is not None:
            self.inference_thread.join(timeout=5.0)

        cv2.destroyAllWindows()

    # Public episode/reset controls

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        if not enabled:
            self.trajectory.clear()
            self.pixel_samples.clear()
            self.sampled_label = None

    def reset_episode(self) -> None:
        self.state = RobotState.SEARCH
        self.trajectory.clear()
        self.pixel_samples.clear()
        self.sampled_label = None
        self.target_xyz = None
        self.target_color = None
        self.gripper_pos = GRIPPER_OPEN

    # Observation extraction
    def _extract_current_joints(self, obs: dict[str, Any]) -> np.ndarray:
        # Current/raw LeRobot layout.
        flat_keys = [f"{joint}.pos" for joint in ARM_JOINTS]
        if all(key in obs for key in flat_keys):
            return np.array(
                [float(obs[key]) for key in flat_keys],
                dtype=float,
            )

        # Older/processed layout.
        if "observation.state" in obs:
            state = to_numpy(obs["observation.state"]).reshape(-1)
            if len(state) < len(ARM_JOINTS):
                raise ValueError(
                    "observation.state does not contain enough joint values."
                )
            return state[: len(ARM_JOINTS)].astype(float)

        raise KeyError(
            "Could not find arm state. Expected flat '*.pos' keys or "
            "'observation.state'. Available keys: "
            f"{list(obs.keys())}"
        )

    def _extract_top_image(self, obs: dict[str, Any]) -> np.ndarray:
        # Current/raw LeRobot layout.
        if "top" in obs:
            return image_to_rgb_uint8(obs["top"])

        # Older/processed layout.
        if "observation.images.top" in obs:
            return image_to_rgb_uint8(obs["observation.images.top"])

        raise KeyError(
            "Could not find top camera image. Expected 'top' or "
            "'observation.images.top'. Available keys: "
            f"{list(obs.keys())}"
        )

    # Action and trajectory helpers
    def _format_action(
            self,
            joints: np.ndarray,
            gripper: float | None = None,
    ) -> dict[str, float]:
        if gripper is None:
            gripper = self.gripper_pos

        action = {
            f"{joint}.pos": float(position)
            for joint, position in zip(ARM_JOINTS, joints)
        }
        action["gripper.pos"] = float(gripper)
        return action

    def _queue_point(
            self,
            joints: np.ndarray,
            gripper: float | None = None,
    ) -> None:
        if gripper is None:
            gripper = self.gripper_pos

        self.trajectory.append(
            {
                "joints": np.asarray(joints, dtype=float).copy(),
                "gripper": float(gripper),
            }
        )

    def _conversion(self, u: float, v: float) -> tuple[float, float]:
        point = np.array([[[u, v]]], dtype=np.float32)
        result = cv2.perspectiveTransform(point, self.H)[0][0]

        if not np.all(np.isfinite(result)):
            raise RuntimeError(
                f"Homography returned invalid coordinates for ({u}, {v})."
            )

        return float(result[0]), float(result[1])

    def _joints_to_ik_vector(self, current_deg: np.ndarray) -> np.ndarray:
        values = np.zeros(len(self.chain.links), dtype=float)

        for joint_name, angle_deg in zip(ARM_JOINTS, current_deg):
            index = self.link_indices[joint_name]
            value_rad = float(np.deg2rad(angle_deg))

            lower, upper = self.chain.links[index].bounds
            epsilon = 1e-6

            if lower is not None:
                value_rad = max(value_rad, lower + epsilon)
            if upper is not None:
                value_rad = min(value_rad, upper - epsilon)

            values[index] = value_rad

        return values

    def _solve_ik(
            self,
            current_deg: np.ndarray,
            target_xyz: np.ndarray,
    ) -> np.ndarray:
        target_xyz = np.asarray(target_xyz, dtype=float)

        if target_xyz.shape != (3,) or not np.all(np.isfinite(target_xyz)):
            raise ValueError(f"Invalid IK target: {target_xyz}")

        initial_position = self._joints_to_ik_vector(current_deg)

        solution = self.chain.inverse_kinematics(
            target_position=target_xyz,
            initial_position=initial_position,
            max_iter=200,
        )

        if not np.all(np.isfinite(solution)):
            raise RuntimeError("IKPy returned non-finite values.")

        solved_pose = self.chain.forward_kinematics(solution)
        solved_xyz = solved_pose[:3, 3]
        error_m = float(np.linalg.norm(solved_xyz - target_xyz))

        print(
            f"IK target={np.round(target_xyz, 4)}, "
            f"solved={np.round(solved_xyz, 4)}, "
            f"error={error_m * 1000:.1f} mm"
        )

        if error_m > MAX_IK_ERROR_M:
            raise RuntimeError(
                "IK position error too large: "
                f"{error_m * 1000:.1f} mm for target {target_xyz}"
            )

        target_deg = np.array(
            [
                np.rad2deg(solution[self.link_indices[joint]])
                for joint in ARM_JOINTS
            ],
            dtype=float,
        )

        # Keep wrist roll fixed because it is inactive in IK.
        target_deg[ARM_JOINTS.index("wrist_roll")] = current_deg[
            ARM_JOINTS.index("wrist_roll")
        ]

        return target_deg

    def _plan_move(
            self,
            target_xyz: np.ndarray,
            duration: float = MOVE_DURATION,
    ) -> None:
        start_joints = self.current_joints.copy()
        target_deg = self._solve_ik(start_joints, target_xyz)

        steps = max(1, int(duration * FPS))

        for step in range(1, steps + 1):
            t = step / steps
            alpha = 3 * t**2 - 2 * t**3
            interpolated = (
                    start_joints
                    + alpha * (target_deg - start_joints)
            )
            self._queue_point(interpolated)

        self.current_joints = target_deg.copy()

    def _plan_wait(self, duration: float) -> None:
        steps = max(1, int(duration * FPS))
        for _ in range(steps):
            self._queue_point(self.current_joints)

    # Detection and display
    def _draw_detections(
            self,
            frame_bgr: np.ndarray,
            detections: list[dict[str, Any]],
            selected: dict[str, Any] | None = None,
    ) -> None:
        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            u, v = det["u"], det["v"]

            cv2.rectangle(
                frame_bgr,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )
            cv2.circle(
                frame_bgr,
                (int(u), int(v)),
                5,
                (0, 0, 255),
                -1,
            )
            cv2.putText(
                frame_bgr,
                f'{det["label"]} {det["conf"]:.2f} {det["dist"]:.3f}m',
                (x1, max(18, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

        if selected is not None:
            x1, y1, x2, y2 = selected["bbox"]
            selected_color = (
                selected["label"]
                .replace(" cube", "")
                .strip()
                .lower()
            )

            cv2.rectangle(
                frame_bgr,
                (x1, y1),
                (x2, y2),
                (255, 0, 255),
                4,
            )
            cv2.putText(
                frame_bgr,
                f"HIGHEST: {selected_color.upper()}",
                (x1, max(25, y1 - 25)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 255),
                2,
            )

    def _show(self, frame_bgr: np.ndarray) -> None:
        cv2.putText(
            frame_bgr,
            f"STATE: {self.state.name}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 255),
            2,
        )
        cv2.imshow("YOLO Cube Detection", frame_bgr)
        cv2.waitKey(1)

    # Main teleoperator action function
    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise RuntimeError("YOLO expert is not connected.")
        obs = self.robot.get_observation()

        measured_joints = self._extract_current_joints(obs)
        frame_rgb = self._extract_top_image(obs)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        with self.lock:
            self.latest_rgb = frame_bgr.copy()
            detections = [dict(det) for det in self.latest_detections]

        # During manual environment reset, hold the current hardware position.
        if not self._enabled:
            self.current_joints = measured_joints
            self._draw_detections(frame_bgr, detections)
            self._show(frame_bgr)
            return self._format_action(measured_joints)

        # Keep the planned trajectory internally consistent. Only replace
        # current_joints with hardware state when no trajectory is active.
        if not self.trajectory:
            self.current_joints = measured_joints

        if self.trajectory:
            point = self.trajectory.pop(0)
            self._draw_detections(frame_bgr, detections)
            self._show(frame_bgr)
            return self._format_action(
                point["joints"],
                point["gripper"],
            )

        selected: dict[str, Any] | None = None

        # State machine
        if self.state == RobotState.SEARCH:
            if detections:
                # select closest cube
                selected = min(detections, key=lambda det: det["v"])

                selected_color = (
                    selected["label"]
                    .replace(" cube", "")
                    .strip()
                    .lower()
                )

                # Do not mix samples from detections with different labels.
                if self.sampled_label != selected_color:
                    self.sampled_label = selected_color
                    self.pixel_samples.clear()

                self.pixel_samples.append(
                    [float(selected["u"]), float(selected["v"])]
                )

                if len(self.pixel_samples) >= STABLE_SAMPLE_COUNT:
                    samples = np.asarray(self.pixel_samples, dtype=float)
                    stable_u = float(np.median(samples[:, 0]))
                    stable_v = float(np.median(samples[:, 1]))

                    robot_x, robot_y = self._conversion(
                        stable_u,
                        stable_v,
                    )

                    self.target_xyz = np.array(
                        [robot_x, robot_y, HOVER_HEIGHT],
                        dtype=float,
                    )

                    self.target_color = selected_color

                    print(
                        f"Selected {self.target_color} cube at "
                        f"pixel=({stable_u:.1f}, {stable_v:.1f}), "
                        f"robot_xy=({robot_x:.4f}, {robot_y:.4f})"
                    )

                    self.pixel_samples.clear()
                    self.sampled_label = None

                    self.state = RobotState.MOVE_ABOVE
                    self._plan_move(
                        np.array(
                            [
                                robot_x - 0.005,
                                robot_y + 0.010,
                                HOVER_HEIGHT,
                                ]
                        )
                    )

        elif self.state == RobotState.MOVE_ABOVE:
            if self.target_xyz is None:
                raise RuntimeError("MOVE_ABOVE entered without target_xyz.")

            self.state = RobotState.LOWER
            self.gripper_pos = GRIPPER_OPEN
            self._plan_move(
                np.array(
                    [
                        self.target_xyz[0] - 0.005,
                        self.target_xyz[1] + 0.007,
                        CUBE_HEIGHT,
                        ]
                )
            )

        elif self.state == RobotState.LOWER:
            self.state = RobotState.GRASP
            self.gripper_pos = GRIPPER_CLOSED
            self._plan_wait(2.0)

        elif self.state == RobotState.GRASP:
            if self.target_xyz is None:
                raise RuntimeError("GRASP entered without target_xyz.")

            self.state = RobotState.LIFT
            self._plan_move(
                np.array(
                    [
                        self.target_xyz[0],
                        self.target_xyz[1],
                        HOVER_HEIGHT,
                    ]
                )
            )

        elif self.state == RobotState.LIFT:
            self.state = RobotState.MOVE_TO_BIN

            if self.target_color == "red":
                hover_target = np.array([0.0439, 0.0890, HOVER_HEIGHT])
                drop_target = np.array([0.0439, 0.0890, DROP_HEIGHT])
            elif self.target_color == "yellow":
                hover_target = np.array([0.0505, 0.2786, HOVER_HEIGHT])
                drop_target = np.array([0.0505, 0.2786, DROP_HEIGHT])
            else:
                hover_target = np.array([0.0505, 0.2786, HOVER_HEIGHT])
                drop_target = np.array([0.0505, 0.2786, DROP_HEIGHT])

            self._plan_move(hover_target)
            self._plan_move(drop_target)

        elif self.state == RobotState.MOVE_TO_BIN:
            self.state = RobotState.OPEN
            self.gripper_pos = GRIPPER_OPEN
            self._plan_wait(1.0)

        elif self.state == RobotState.OPEN:
            self.state = RobotState.RETURN
            if self.target_color == "red":
                hover_target = np.array([0.05, 0.2786, HOVER_HEIGHT])
            elif self.target_color == "yellow":
                hover_target = np.array([0.0505, 0.2786, HOVER_HEIGHT])
            else:
                hover_target = np.array([0.0505, 0.2786, HOVER_HEIGHT])

            self._plan_move(hover_target)
            self._plan_move(np.array([0.08, 0.08, 0.1]))

        elif self.state == RobotState.RETURN:
            self.state = RobotState.SEARCH
            self.target_xyz = None
            self.target_color = None
            self.gripper_pos = GRIPPER_OPEN
            self._plan_wait(1.0)

        self._draw_detections(frame_bgr, detections, selected)
        self._show(frame_bgr)

        if self.trajectory:
            point = self.trajectory.pop(0)
            return self._format_action(
                point["joints"],
                point["gripper"],
            )
        return self._format_action(self.current_joints)

    # Background YOLO worker
    def _inference_worker(self) -> None:
        while self.running:
            with self.lock:
                frame = (
                    None
                    if self.latest_rgb is None
                    else self.latest_rgb.copy()
                )

            if frame is None:
                time.sleep(0.01)
                continue

            try:
                results = self.model.predict(
                    frame,
                    conf=MIN_DETECTION_CONF,
                    verbose=False,
                )

                current_detections: list[dict[str, Any]] = []

                if results and results[0].boxes is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = (
                            box.xyxy[0]
                            .detach()
                            .cpu()
                            .numpy()
                            .astype(float)
                        )

                        u = float((x1 + x2) / 2.0)
                        v = float((y1 + y2) / 2.0)

                        robot_x, robot_y = self._conversion(u, v)
                        distance = float(np.hypot(robot_x, robot_y))

                        class_id = int(
                            box.cls[0].detach().cpu().item()
                        )
                        confidence = float(
                            box.conf[0].detach().cpu().item()
                        )
                        label = str(results[0].names[class_id])

                        current_detections.append(
                            {
                                "u": u,
                                "v": v,
                                "rx": robot_x,
                                "ry": robot_y,
                                "dist": distance,
                                "label": label,
                                "conf": confidence,
                                "bbox": (
                                    int(x1),
                                    int(y1),
                                    int(x2),
                                    int(y2),
                                ),
                            }
                        )

                with self.lock:
                    self.latest_detections = current_detections

            except Exception:
                logging.exception("YOLO inference worker failed.")
                time.sleep(0.1)

# main
def main() -> None:
    robot_config = SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        use_degrees=True,
        cameras={
            "top": OpenCVCameraConfig(
                index_or_path=CAMERA_INDEX,
                width=640,
                height=480,
                fps=FPS,
            )
        },
    )

    robot = SO101Follower(robot_config)
    teleop = YOLOExpertTeleop(robot)

    # Dataset feature construction used by the LeRobot
    action_features = hw_to_dataset_features(
        robot.action_features,
        "action",
    )
    observation_features = hw_to_dataset_features(
        robot.observation_features,
        "observation",
    )
    dataset_features = {
        **action_features,
        **observation_features,
    }

    dataset = LeRobotDataset.create(
        repo_id=REPO_ID,
        fps=FPS,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4,
    )

    listener = None
    events = None

    teleop_action_processor, robot_action_processor, robot_observation_processor = (
        make_default_processors()
    )

    try:
        init_visualization("rerun", session_name="recording")

        # Match current LeRobot's preferred connection order.
        teleop.connect()
        robot.connect()

        listener, events = init_keyboard_listener()

        episode_idx = 0

        while (
            episode_idx < NUM_EPISODES
            and not events["stop_recording"]
        ):
            teleop.set_enabled(True)
            teleop.reset_episode()

            log_say(
                f"Recording episode {episode_idx + 1} "
                f"of {NUM_EPISODES}"
            )

            record_loop(
                robot=robot,
                events=events,
                fps=FPS,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                teleop=teleop,
                dataset=dataset,
                control_time_s=EPISODE_TIME_SEC,
                single_task=TASK_DESCRIPTION,
                display_data=True,
            )

            should_reset = (
                not events["stop_recording"]
                and (
                    episode_idx < NUM_EPISODES - 1
                    or events["rerecord_episode"]
                )
            )

            if should_reset:
                log_say("Reset the environment")

                # Keep passing a real teleoperator so record_loop advances,
                # but disable the state machine so the arm only holds position.
                teleop.set_enabled(False)

                record_loop(
                    robot=robot,
                    events=events,
                    fps=FPS,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop,
                    dataset=None,
                    control_time_s=RESET_TIME_SEC,
                    single_task=TASK_DESCRIPTION,
                    display_data=True,
                )

                teleop.set_enabled(True)

            if events["rerecord_episode"]:
                log_say("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            if events["stop_recording"]:
                break

            dataset.save_episode()
            episode_idx += 1

        log_say("Finalizing dataset...")
        dataset.finalize()

        if dataset.num_episodes > 0:
            dataset.push_to_hub()
        else:
            logging.warning(
                "No episodes were saved; skipping Hub upload."
            )

    finally:
        log_say("Stopping recording")

        try:
            if listener is not None:
                listener.stop()
        except Exception:
            logging.exception("Could not stop keyboard listener.")

        try:
            if teleop.is_connected:
                teleop.disconnect()
        except Exception:
            logging.exception("Could not disconnect teleoperator.")

        try:
            if robot.is_connected:
                robot.disconnect()
        except Exception:
            logging.exception("Could not disconnect robot.")

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
 