import time
from pathlib import Path

import numpy as np
from ikpy.chain import Chain
from ikpy.urdf import URDF

from lerobot.robots.so_follower import (
    SO101Follower,
    SO101FollowerConfig,
)



PORT = "COM8"
ROBOT_ID = "like_and_follow_er_arm"

# Location of the SO101 URDF
URDF_PATH = (
    Path(__file__).parent
    / "SO101"
    / "so101_new_calib.urdf"
)

# Desired gripper-tip XYZ position
TARGET_X = 0.20
TARGET_Y = 0.15
TARGET_Z = 0.1

# How long the movement should take
MOVE_DURATION = 5.0

# Command rate
FPS = 50

# Reject IK solutions farther than this from requested XYZ
MAX_IK_ERROR = 0.01       # 1 cm

# Reject suspicious IK solutions requiring a huge joint movement
MAX_JOINT_CHANGE = 90.0   # degrees


ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def create_ik_chain():
    if not URDF_PATH.exists():
        raise FileNotFoundError(
            f"URDF not found:\n{URDF_PATH}"
        )

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
        link.name in ARM_JOINTS
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


def get_current_arm_joints(robot):

    observation = robot.get_observation()

    joints = np.array(
        [
            observation[f"{joint}.pos"]
            for joint in ARM_JOINTS
        ],
        dtype=float,
    )

    return joints


def joints_to_ik_vector(chain, joint_degrees):
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

        lower, upper = chain.links[index].bounds
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

        if not np.isclose(raw_rad, value_rad):

            print(
                f"WARNING: {joint_name} current position "
                f"{angle_deg:.2f}° is outside URDF limits."
            )

            print(
                f"         IK initial guess clamped to "
                f"{np.rad2deg(value_rad):.2f}°"
            )

        values[index] = value_rad

    return values

def ik_vector_to_joints(chain, solution):
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

def move_joints_smoothly(
    robot,
    current_deg,
    target_deg,
    duration=MOVE_DURATION,
    fps=FPS,
):

    steps = max(1, int(duration * fps))

    print("\nMoving arm...")

    for step in range(steps + 1):

        t = step / steps

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

def move_xyz(
    robot,
    chain,
    x,
    y,
    z,
    duration=MOVE_DURATION,
):

    current_deg = get_current_arm_joints(robot)

    print("\nCurrent joint angles:")

    for name, angle in zip(
        ARM_JOINTS,
        current_deg,
    ):
        print(f"  {name:15s}: {angle:8.2f}°")

    initial_position = joints_to_ik_vector(
        chain,
        current_deg,
    )

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

    solution = chain.inverse_kinematics(
        target_position=target_xyz,
        initial_position=initial_position,
        max_iter=200,
    )

    target_deg = ik_vector_to_joints(
        chain,
        solution,
    )

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

    if error > MAX_IK_ERROR:

        raise RuntimeError(
            "IK could not accurately reach the requested "
            f"position.\nError = {error * 1000:.1f} mm"
        )

    print("\nTarget joint angles:")

    for name, angle in zip(
        ARM_JOINTS,
        target_deg,
    ):
        print(f"  {name:15s}: {angle:8.2f}°")

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

    move_joints_smoothly(
        robot,
        current_deg,
        target_deg,
        duration=duration,
    )

def set_gripper(robot, position):
    position = max(0.0, min(100.0, position))

    robot.send_action({
        "gripper.pos": float(position)
    })

def main():

    chain = create_ik_chain()

    config = SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        use_degrees=True,
    )

    robot = SO101Follower(config)

    try:

        print("\nConnecting to SO101...")

        robot.connect()

        print("SO101 connected.")
        set_gripper(robot, 40)
        move_xyz(
            robot,
            chain,
            x=TARGET_X,
            y=TARGET_Y,
            z=TARGET_Z,
            duration=MOVE_DURATION,
        )
        set_gripper(robot, 20)
        time.sleep(2.0)
    finally:

        if robot.is_connected:
            robot.disconnect()

        print("\nRobot disconnected.")


if __name__ == "__main__":
    main()