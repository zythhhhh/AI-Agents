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
# USER SETTINGS
# ============================================================

# Change this to your follower's actual COM port
PORT = "COM8"

# IMPORTANT:
# Use the SAME robot ID that you used when calibrating the follower.
ROBOT_ID = "like_and_follow_er_arm"

# Location of the SO101 URDF
URDF_PATH = (
    Path(__file__).parent
    / "SO101"
    / "so101_new_calib.urdf"
)

# Desired gripper-tip XYZ position, in METERS,
# relative to the robot's base coordinate system.
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


# ============================================================
# ARM JOINTS
# ============================================================

ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


# ============================================================
# CREATE IKPY CHAIN
# ============================================================

def create_ik_chain():
    """
    Create an IKPy chain from the SO101 base to gripper_frame_link.
    """

    if not URDF_PATH.exists():
        raise FileNotFoundError(
            f"URDF not found:\n{URDF_PATH}"
        )

    # Force IKPy to follow:
    #
    # base
    #   ↓
    # shoulder_pan
    # shoulder_lift
    # elbow_flex
    # wrist_flex
    # wrist_roll
    #   ↓
    # gripper_frame_joint
    #
    # We specifically DO NOT include the moving gripper jaw.

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


# ============================================================
# CURRENT JOINT STATE
# ============================================================

def get_current_arm_joints(robot):
    """
    Read the SO101 arm joint positions from LeRobot.

    Returns:
        NumPy array in DEGREES.
    """

    observation = robot.get_observation()

    joints = np.array(
        [
            observation[f"{joint}.pos"]
            for joint in ARM_JOINTS
        ],
        dtype=float,
    )

    return joints


# ============================================================
# CONVERT LEROBOT JOINTS -> IKPY VECTOR
# ============================================================

def joints_to_ik_vector(chain, joint_degrees):
    """
    Convert LeRobot joint angles (degrees) into IKPy's full
    joint vector (radians).

    If the calibrated robot reports an angle slightly outside
    the generic URDF limits, clamp ONLY the IK solver's initial
    guess to the allowed URDF range.
    """

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
                f"         IK initial guess clamped to "
                f"{np.rad2deg(value_rad):.2f}°"
            )

        values[index] = value_rad

    return values

def ik_vector_to_joints(chain, solution):
    """
    Extract the five SO101 arm joints from the complete
    IKPy solution and convert radians -> degrees.
    """

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


# ============================================================
# SMOOTH MOTION
# ============================================================

def move_joints_smoothly(
    robot,
    current_deg,
    target_deg,
    duration=MOVE_DURATION,
    fps=FPS,
):
    """
    Smoothly interpolate from the current joint configuration
    to the target configuration.

    Uses smoothstep so velocity starts and ends gently.
    """

    steps = max(1, int(duration * fps))

    print("\nMoving arm...")

    for step in range(steps + 1):

        t = step / steps

        # Smoothstep interpolation
        #
        # 0 -> gradually accelerate
        # middle -> fastest
        # 1 -> gradually decelerate

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


# ============================================================
# MOVE GRIPPER TIP TO XYZ
# ============================================================

def move_xyz(
    robot,
    chain,
    x,
    y,
    z,
    duration=MOVE_DURATION,
):
    """
    Move the SO101 gripper tip to an XYZ coordinate.

    XYZ is in meters relative to the SO101 URDF base frame.
    """

    # --------------------------------------------------------
    # Read current arm position
    # --------------------------------------------------------

    current_deg = get_current_arm_joints(robot)

    print("\nCurrent joint angles:")

    for name, angle in zip(
        ARM_JOINTS,
        current_deg,
    ):
        print(f"  {name:15s}: {angle:8.2f}°")

    # --------------------------------------------------------
    # Convert current robot state for IKPy
    # --------------------------------------------------------

    initial_position = joints_to_ik_vector(
        chain,
        current_deg,
    )

    # --------------------------------------------------------
    # Calculate current XYZ
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # Target
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # INVERSE KINEMATICS
    # --------------------------------------------------------

    solution = chain.inverse_kinematics(
        target_position=target_xyz,
        initial_position=initial_position,
        max_iter=200,
    )

    target_deg = ik_vector_to_joints(
        chain,
        solution,
    )
    # --------------------------------------------------------
    # Verify resulting XYZ using forward kinematics
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # Safety check #1
    # --------------------------------------------------------

    if error > MAX_IK_ERROR:

        raise RuntimeError(
            "IK could not accurately reach the requested "
            f"position.\nError = {error * 1000:.1f} mm"
        )

    # --------------------------------------------------------
    # Print calculated joint target
    # --------------------------------------------------------

    print("\nTarget joint angles:")

    for name, angle in zip(
        ARM_JOINTS,
        target_deg,
    ):
        print(f"  {name:15s}: {angle:8.2f}°")

    # --------------------------------------------------------
    # Safety check #2
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # ACTUALLY MOVE
    # --------------------------------------------------------

    move_joints_smoothly(
        robot,
        current_deg,
        target_deg,
        duration=duration,
    )

def set_gripper(robot, position):
    """
    position:
        0   = closed
        100 = open
    """

    position = max(0.0, min(100.0, position))

    robot.send_action({
        "gripper.pos": float(position)
    })

# ============================================================
# MAIN
# ============================================================

def main():
    # --------------------------------------------------------
    # Create IK solver
    # --------------------------------------------------------

    chain = create_ik_chain()

    # --------------------------------------------------------
    # Configure SO101
    # --------------------------------------------------------

    config = SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        use_degrees=True,
    )

    robot = SO101Follower(config)

    try:

        # ----------------------------------------------------
        # Connect to physical arm
        # ----------------------------------------------------

        print("\nConnecting to SO101...")

        robot.connect()

        print("SO101 connected.")
        set_gripper(robot, 40)
        # ----------------------------------------------------
        # GO TO XYZ
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Always disconnect, even if IK fails
        # ----------------------------------------------------

        if robot.is_connected:
            robot.disconnect()

        print("\nRobot disconnected.")


if __name__ == "__main__":
    main()