# imports for YOLOWORLD
import cv2
from ultralytics import YOLOWorld

# imports for moving arm using IK
import time
from pathlib import Path

import numpy as np
from ikpy.chain import Chain
from ikpy.urdf import URDF

from lerobot.robots.so_follower import (
    SO101Follower,
    SO101FollowerConfig,
)

# CONSTANTS
HOVER_HEIGHT = 0.10
CUBE_HEIGHT = 0.005
DROPX = 0.30
DROPY = 0.10
DROPZ = 0.10

# set up YOLO
model = YOLOWorld("yolov8s-worldv2.pt")

classes = [
    "red cube",
    "blue cube",
    "yellow cube",
    "green cube",
]

model.set_classes(classes)

# set up camera
cap = cv2.VideoCapture(1) # Change to your camera index

if not cap.isOpened():
    raise RuntimeError("Could not open camera")

target_color = input("Which cube? red / blue / yellow: ").strip().lower()
target_label = f"{target_color} cube"

# set up arm configs
PORT = "COM8" # Change this to your follower arm COM port
ROBOT_ID = "like_and_follow_er_arm" # Use the robot ID when calibrating the follower.

URDF_PATH = (
    Path(__file__).parent
    / "SO101"
    / "so101_new_calib.urdf"
    # Location of the SO101 URDF
)

MOVE_DURATION = 2.0 # Duration of movement
FPS = 50 # Command rate

# Reject IK solutions farther than this from requested XYZ
MAX_IK_ERROR = 0.01       # 1 cm

# Reject suspicious IK solutions requiring a huge joint movement
MAX_JOINT_CHANGE = 90.0   # degrees

# define arm joints
ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
IK_ACTIVE_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
]

# define the conversion from pixels to robot distance
def conversion (u,v):
    pixel_points = np.array([
        [595,  40],   # TL
        [604, 437],   # TR
        [ 78,  33],   # BL
        [ 71, 437],   # BR
    ], dtype=np.float32)

    robot_points = np.array([
        [0.4350,  0.0291],   # TL
        [0.4465, -0.1305],   # TR
        [0.1409,  0.0141],   # BL
        [0.1524, -0.1455],   # BR
    ], dtype=np.float32)


    # Calculate transformation
    H, _ = cv2.findHomography(
        pixel_points,
        robot_points
    )

    print("Homography matrix:")
    print(H)

    point = np.array(
        [[[u, v]]],
        dtype=np.float32
    )

    result = cv2.perspectiveTransform(
        point, H)[0][0]

    return float(result[0]), float(result[1])

# define ikpy 
def create_ik_chain():
    # Create an IKPy chain from the SO101 base to gripper_frame_link.

    if not URDF_PATH.exists():
        raise FileNotFoundError(
            f"URDF not found:\n{URDF_PATH}"
        )

    # use following pipeline to chain
    # base -> shoulder_pan -> shoulder_lift -> elbow_flex -> wrist_flex -> wrist_roll -> gripper_frame_joint
    # We specifically DO NOT include the moving gripper jaw

    joint_path = ARM_JOINTS + ["gripper_frame_joint"]

    base_elements = URDF.get_chain_from_joints(
        str(URDF_PATH),
        joint_path,
    )

    # Load once so we can determine exactly where each joint
    # appears in IKPy's chain.
    temp_chain = Chain.from_urdf_file(
        str(URDF_PATH),
        base_elements=base_elements,
    )

    # Only the five arm joints should be adjustable by IK.
    active_mask = [
        link.name in IK_ACTIVE_JOINTS
        for link in temp_chain.links
    ]

    chain = Chain.from_urdf_file(
        str(URDF_PATH),
        base_elements=base_elements,
        active_links_mask=active_mask,
    )

    print("\nIK chain:")

    for i, link in enumerate(chain.links):
        status = "ACTIVE" if chain.active_links_mask[i] else "fixed"
        print(f"  {i}: {link.name:25s} {status}")

    return chain

# define current joint state
def get_current_arm_joints(robot):

    # Reads the SO101 arm joint positions from LeRobot.
    # Returns NumPy array in DEGREES.

    observation = robot.get_observation()

    joints = np.array(
        [
            observation[f"{joint}.pos"]
            for joint in ARM_JOINTS
        ],
        dtype=float,
    )

    return joints

# convert lerobot joints to IKPY joint vector
def joints_to_ik_vector(chain, joint_degrees):
    # Convert LeRobot joint angles (degrees) into IKPy's full joint vector (radians).
    # If the calibrated robot exceeds the generic URDF limits, clamp the IK solver's initial guess

    values = np.zeros(len(chain.links))

    link_indices = {
        link.name: i
        for i, link in enumerate(chain.links)
    }

    for joint_name, angle_deg in zip(
        ARM_JOINTS,
        joint_degrees,
    ):
        index = link_indices[joint_name]

        raw_rad = np.deg2rad(angle_deg)
        value_rad = raw_rad

        # IKPy stores URDF joint limits here
        lower, upper = chain.links[index].bounds

        # Tiny margin so scipy starts INSIDE the limits
        epsilon = 1e-6

        if lower is not None:
            value_rad = max(
                value_rad,
                lower + epsilon,
            )

        if upper is not None:
            value_rad = min(
                value_rad,
                upper - epsilon,
            )

        # Tell us if we had to clamp anything
        if not np.isclose(raw_rad, value_rad):

            print(
                f"WARNING: {joint_name} current position "
                f"{angle_deg:.2f}° is outside URDF limits."
            )

            print(
                f" IK initial guess clamped to "
                f"{np.rad2deg(value_rad):.2f}°"
            )

        values[index] = value_rad

    return values

# convert IKPY joint vector back to lerobot joints 
def ik_vector_to_joints(chain, solution):
    # Extract the five SO101 arm joints from the IKPy solution and convert radians -> degrees

    link_indices = {
        link.name: i
        for i, link in enumerate(chain.links)
    }

    degrees = np.array(
        [
            np.rad2deg(
                solution[link_indices[joint]]
            )
            for joint in ARM_JOINTS
        ]
    )

    return degrees

# ensure smooth joint motion
def move_joints_smoothly(
    robot,
    current_deg,
    target_deg,
    duration=MOVE_DURATION,
    fps=FPS,
):
    # Smoothly interpolate from the current joint configuration to the target configuration.
    # Uses smoothstep so velocity starts and ends gently.

    steps = max(1, int(duration * fps))

    print("\nMoving arm...")

    for step in range(steps + 1):

        t = step / steps

        # Smoothstep interpolation
        # gradually accelerate, fastest, gradually decelerate

        alpha = 3 * t**2 - 2 * t**3

        joint_positions = (
            current_deg
            + alpha * (target_deg - current_deg)
        )

        action = {
            f"{joint}.pos": float(position)
            for joint, position in zip(
                ARM_JOINTS,
                joint_positions,
            )
        }

        robot.send_action(action)

        time.sleep(1.0 / fps)

    print("Movement complete.")


# function to move the SO101 gripper tip to an XYZ coordinate.
def move_xyz(
    robot,
    chain,
    x,
    y,
    z,
    duration=MOVE_DURATION,
):

    # read current arm
    current_deg = get_current_arm_joints(robot)

    print("\nCurrent joint angles:")

    for name, angle in zip(
        ARM_JOINTS,
        current_deg,
    ):
        print(f"  {name:15s}: {angle:8.2f}°")

    # Convert current robot state for IKPy
    initial_position = joints_to_ik_vector(
        chain,
        current_deg,
    )

    # Calculate current XYZ
    current_pose = chain.forward_kinematics(
        initial_position
    )

    current_xyz = current_pose[:3, 3]

    print("\nCurrent gripper XYZ:")
    print(
        f"  X = {current_xyz[0]:.4f} m\n"
        f"  Y = {current_xyz[1]:.4f} m\n"
        f"  Z = {current_xyz[2]:.4f} m"
    )

    # Target
    target_xyz = np.array(
        [x, y, z],
        dtype=float,
    )

    print("\nRequested target XYZ:")
    print(
        f"  X = {x:.4f} m\n"
        f"  Y = {y:.4f} m\n"
        f"  Z = {z:.4f} m"
    )

    # INVERSE KINEMATICS
    solution = chain.inverse_kinematics(
        target_position=target_xyz,
        initial_position=initial_position,
        max_iter=200,
    )

    target_deg = ik_vector_to_joints(
        chain,
        solution,
    )

    # Verify resulting XYZ using forward kinematics
    solved_pose = chain.forward_kinematics(
        solution
    )

    solved_xyz = solved_pose[:3, 3]

    error = np.linalg.norm(
        solved_xyz - target_xyz
    )

    print("\nIK solution XYZ:")
    print(
        f"  X = {solved_xyz[0]:.4f} m\n"
        f"  Y = {solved_xyz[1]:.4f} m\n"
        f"  Z = {solved_xyz[2]:.4f} m"
    )

    print(
        f"\nIK position error: "
        f"{error * 1000:.2f} mm"
    )

    # Safety check if IK and FK have too big disrepcancy
    if error > MAX_IK_ERROR:

        raise RuntimeError(
            "IK could not accurately reach the requested "
            f"position.\nError = {error * 1000:.1f} mm"
        )

    # Print calculated joint target
    print("\nTarget joint angles:")

    for name, angle in zip(
        ARM_JOINTS,
        target_deg,
    ):
        print(f"  {name:15s}: {angle:8.2f}°")

    # Safety check if joint angles are too large
    joint_changes = np.abs(
        target_deg - current_deg
    )

    print("\nJoint changes:")

    for name, change in zip(
        ARM_JOINTS,
        joint_changes,
    ):
        print(f"  {name:15s}: {change:8.2f}°")

    if np.any(
        joint_changes > MAX_JOINT_CHANGE
    ):

        raise RuntimeError(
            "IK produced a suspiciously large joint movement. "
            "Arm will NOT move."
        )

    # actually move
    move_joints_smoothly(
        robot,
        current_deg,
        target_deg,
        duration=duration,
    )

def set_gripper(robot, position):
    # position 0 is closed, 100 is fully open

    position = max(0.0, min(100.0, position))

    robot.send_action({
        "gripper.pos": float(position)
    })
def main():
    # create IKPY chain
    chain = create_ik_chain()

    # configurate SO101 arm
    config = SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        use_degrees=True,
    )

    robot = SO101Follower(config)

    connected = False
    
    # try to connect and run arm
    try:
        # Connect to physical arm
        print("\nConnecting to SO101...")

        robot.connect()

        print("SO101 connected.")

        # use a buffer of 10 samples to ensure accuracy
        pixel_samples=[]
        # while loop to run YOLO
        while True:
            ok, frame = cap.read()

            if not ok:
                continue

            # Run YOLO
            results = model.predict(
                frame,
                conf=0.20,
                verbose=False,
            )

            result = results[0]

            detections = []

            for box in result.boxes:
                # Bounding box corners
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                confidence = float(box.conf[0])
                class_id = int(box.cls[0])

                label = result.names[class_id]

                # Center of bounding box
                u = (x1 + x2) / 2
                v = (y1 + y2) / 2

                detections.append({
                    "label": label,
                    "confidence": confidence,
                    "u": float(u),
                    "v": float(v),
                    "bbox": (int(x1), int(y1), int(x2), int(y2)),
                })

                # Draw visualization
                cv2.rectangle(
                    frame,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2,
                )

                cv2.circle(
                    frame,
                    (int(u), int(v)),
                    5,
                    (0, 0, 255),
                    -1,
                )

                cv2.putText(
                    frame,
                    f"{label} {confidence:.2f}",
                    (int(x1), int(y1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

            # Find requested cube
            target_detections = [
                d for d in detections
                if d["label"] == target_label
            ]

            if target_detections:
                # Use highest-confidence detection
                target = max(
                    target_detections,
                    key=lambda x: x["confidence"],
                )

                print(
                    f"{target['label']}: "
                    f"pixel=({target['u']:.1f}, {target['v']:.1f}), "
                    f"confidence={target['confidence']:.2f}"
                )

                # append sample to buffer
                pixel_samples.append(
                    [target['u'],
                     target['v'],
                ])

                # find median of samples to use
                if len(pixel_samples) >= 10:
                    samples = np.array(pixel_samples)

                    u = np.median(samples[:, 0])
                    v = np.median(samples[:, 1])

                    print(
                        f"Stable pixel location: "
                        f"({u:.1f}, {v:.1f})"
                    )
                    break

            cv2.imshow("YOLO Cube Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return
                
        # convert to robot
        targetx, targety = conversion(u,v)

        # open gripper, move, close gripper
        set_gripper(robot, 45)  # open gripper
        time.sleep(1.0)
        # move
        move_xyz(
            robot,
            chain,
            x=targetx + 0.02,
            y=targety + 0.02,
            z=HOVER_HEIGHT,
            duration=MOVE_DURATION,
        )

        move_xyz(
            robot,
            chain,
            x=targetx + 0.02,
            y=targety + 0.02,
            z=CUBE_HEIGHT,
            duration=MOVE_DURATION,
        )

        set_gripper(robot, 15)  # close gripper
        time.sleep(2.0)  # delay for the gripper to move

        move_xyz(
            robot,
            chain,
            x=targetx,
            y=targety,
            z=HOVER_HEIGHT,
            duration=MOVE_DURATION,
        )

        move_xyz(
            robot,
            chain,
            x=DROPX,
            y=DROPY,
            z=DROPZ,
            duration=MOVE_DURATION,
        )

        set_gripper(robot, 40)  # open gripper
        time.sleep(2.0)  # delay for the gripper to move

        # # return to standard position
        # move_xyz(
        #     robot,
        #     chain,
        #     x=0.1498,
        #     y=-0.0175,
        #     z=-0.0083,
        #     duration=MOVE_DURATION,
        # )

        set_gripper(robot, 0)  # open gripper
        time.sleep(2.0)  # delay for the gripper to move

    finally:
        # disconnect and close windows
        if robot.is_connected:
            robot.disconnect()
        print("\nRobot disconnected.")
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
