import time
from pathlib import Path

import numpy as np
from ikpy.chain import Chain
from ikpy.urdf import URDF

from lerobot.robots.so_follower import (
    SO101Follower,
    SO101FollowerConfig,
)


# ============================================================
# SETTINGS
# ============================================================

PORT = "COM8"
ROBOT_ID = "like_and_follow_er_arm"

URDF_PATH = (
    Path(__file__).parent
    / "SO101"
    / "so101_new_calib.urdf"
)

ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


# ============================================================
# IKPY CHAIN
# ============================================================

def create_ik_chain():

    joint_path = ARM_JOINTS + [
        "gripper_frame_joint"
    ]

    base_elements = URDF.get_chain_from_joints(
        str(URDF_PATH),
        joint_path,
    )

    chain = Chain.from_urdf_file(
        str(URDF_PATH),
        base_elements=base_elements,
        active_links_mask=[
            False,  # Base
            True,   # shoulder_pan
            True,   # shoulder_lift
            True,   # elbow_flex
            True,   # wrist_flex
            True,   # wrist_roll
            False,  # gripper_frame_joint
        ],
    )

    return chain


# ============================================================
# ROBOT JOINTS -> IKPY
# ============================================================

def get_ik_vector(robot, chain):

    obs = robot.get_observation()

    joint_deg = np.array([
        obs[f"{joint}.pos"]
        for joint in ARM_JOINTS
    ])

    values = np.zeros(
        len(chain.links)
    )

    link_indices = {
        link.name: i
        for i, link in enumerate(chain.links)
    }

    for name, angle_deg in zip(
        ARM_JOINTS,
        joint_deg,
    ):

        index = link_indices[name]

        angle_rad = np.deg2rad(
            angle_deg
        )

        # Clamp only for FK/IK compatibility
        lower, upper = chain.links[index].bounds

        if lower is not None:
            angle_rad = max(
                angle_rad,
                lower
            )

        if upper is not None:
            angle_rad = min(
                angle_rad,
                upper
            )

        values[index] = angle_rad

    return values, joint_deg


# ============================================================
# MAIN
# ============================================================

def main():

    chain = create_ik_chain()

    config = SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        use_degrees=True,
    )

    robot = SO101Follower(config)

    try:

        print("Connecting...")
        robot.connect()
        print("Connected.")

        # ----------------------------------------------------
        # IMPORTANT:
        # Disable torque so you can move the arm manually
        # ----------------------------------------------------

        robot.bus.disable_torque()

        print("\nTorque disabled.")
        print("Move the arm by hand.")
        print("Press Ctrl+C to stop.\n")

        while True:

            ik_vector, joints = get_ik_vector(
                robot,
                chain
            )

            pose = chain.forward_kinematics(
                ik_vector
            )

            xyz = pose[:3, 3]

            print(
                f"\r"
                f"X = {xyz[0]: .4f} m   "
                f"Y = {xyz[1]: .4f} m   "
                f"Z = {xyz[2]: .4f} m",
                end="",
                flush=True,
            )

            time.sleep(0.1)

    except KeyboardInterrupt:

        print("\n\nStopped.")

    finally:

        if robot.is_connected:
            robot.disconnect()

        print("Robot disconnected.")


if __name__ == "__main__":
    main()