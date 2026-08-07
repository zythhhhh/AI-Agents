import sys
import time
import serial
import xarm

# CONFIGURATION
PORT = "COM10"
BAUD_RATE = 9600

# Target Zone 1 -> Arm 1 Only
ZONE1_MIN_CM = 15.0
ZONE1_MAX_CM = 25.0

# Target Zone 2 -> Arm 2 Only
ZONE2_MIN_CM = 1.0
ZONE2_MAX_CM = 14.0


DELAY_BEFORE_MOVE_SEC = 0.5  # Delay before initiating pick

# Calibrated claw positions
CLAW_OPEN = 100
CLAW_CLOSE = 1000


def initialize_arms():
    """Initializes arm controllers and returns their instances."""
    print("Connecting to xArm controllers...")
    arm1 = xarm.Controller("USB1")  # Front-Right Arm
    arm2 = xarm.Controller("USB0")  # Back-Left Arm
    return arm1, arm2


def reset_arm1(arm1):
    """Resets Arm 1 to its ready position."""
    print("Setting Arm 1 to Ready Position...")
    arm1.setPosition(1, CLAW_OPEN)
    arm1.setPosition(2, 500)
    arm1.setPosition(3, 330)
    arm1.setPosition(4, 780)
    arm1.setPosition(5, 750)
    arm1.setPosition(6, 860)


def reset_arm2(arm2):
    """Resets Arm 2 to its ready position."""
    print("Setting Arm 2 to Ready Position...")
    arm2.setPosition(1, CLAW_OPEN)
    arm2.setPosition(2, 900)
    arm2.setPosition(3, 480)
    arm2.setPosition(4, 400)
    arm2.setPosition(5, 250)
    arm2.setPosition(6, 100)


def reset_to_master_set_position(arm1, arm2):
    """Moves both arms back to their calibrated ready position."""
    reset_arm1(arm1)
    reset_arm2(arm2)
    time.sleep(2.0)


def execute_arm1_pick_and_place(arm1):
    """Executes pick and place sequence using ONLY Arm 1."""
    print("\n--- Starting Arm 1 Pick and Place Sequence ---")

    # Grab Box
    print("Arm 1: Closing gripper...")
    arm1.setPosition(1, CLAW_CLOSE)
    time.sleep(1.5)

    # Lift
    print("Arm 1: Lifting...")
    arm1.setPosition(3, 10)
    arm1.setPosition(4, 600)
    arm1.setPosition(5, 600)
    time.sleep(1.0)

    # Pivot
    print("Arm 1: Pivoting...")
    arm1.setPosition(5, 550)
    arm1.setPosition(6, 500)
    time.sleep(1.0)

    # Drop Box
    print("Arm 1: Releasing object...")
    arm1.setPosition(1, CLAW_OPEN)
    time.sleep(2.0)

    # Reset to Ready Position
    print("Arm 1: Returning to ready position...")
    arm1.setPosition(6, 880)
    time.sleep(1.0)
    arm1.setPosition(2, 500)
    arm1.setPosition(3, 290)
    arm1.setPosition(4, 800)
    time.sleep(1.0)
    arm1.setPosition(5, 750)
    time.sleep(1.0)
    print("--- Arm 1 Pick and Place Complete ---\n")


def execute_arm2_pick_and_place(arm2):
    """Executes pick and place sequence using ONLY Arm 2."""
    print("\n--- Starting Arm 2 Pick and Place Sequence ---")

    # Grab Box
    print("Arm 2: Closing gripper...")
    arm2.setPosition(1, CLAW_CLOSE)
    time.sleep(1.5)

    # Lift
    print("Arm 2: Lifting...")
    arm2.setPosition(3, 800)
    arm2.setPosition(4, 600)
    arm2.setPosition(5, 430)
    time.sleep(1.0)

    # Pivot
    print("Arm 2: Pivoting...")
    arm2.setPosition(5, 500)
    arm2.setPosition(6, 500)
    time.sleep(1.0)

    # Drop Box
    print("Arm 2: Releasing object...")
    arm2.setPosition(1, CLAW_OPEN)
    time.sleep(2.0)

    # Reset to Ready Position
    print("Arm 2: Returning to ready position...")
    arm2.setPosition(6, 100)
    time.sleep(1.0)
    arm2.setPosition(2, 900)
    arm2.setPosition(3, 480)
    arm2.setPosition(4, 400)
    time.sleep(1.0)
    arm2.setPosition(5, 260)
    arm2.setPosition(6, 100)
    time.sleep(1.0)
    print("--- Arm 2 Pick and Place Complete ---\n")


def parse_distance(line):
    """Safely extracts distance float value from incoming string lines."""
    try:
        if "Distance:" in line:
            parts = line.split("Distance:")
            val_part = parts[1].strip().split()[0]
            return float(val_part)
    except (IndexError, ValueError):
        pass
    return None


def get_active_zone(distance):
    """Determines which zone (1 or 2) the distance falls into."""
    if ZONE1_MIN_CM <= distance <= ZONE1_MAX_CM:
        return 1
    elif ZONE2_MIN_CM <= distance <= ZONE2_MAX_CM:
        return 2
    return None


def main():
    print("--- High-Frequency Python Sensor Listener with Selective xArm Automation ---")
    print(f"Zone 1 ({ZONE1_MIN_CM}-{ZONE1_MAX_CM} cm) -> Triggers Arm 1")
    print(f"Zone 2 ({ZONE2_MIN_CM}-{ZONE2_MAX_CM} cm) -> Triggers Arm 2")

    # Initialize arms
    arm1, arm2 = initialize_arms()
    reset_to_master_set_position(arm1, arm2)

    print(f"Connecting to sensor on {PORT}...")

    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)  # Allow Arduino reboot window

        ser.reset_input_buffer()
        print("Connected! Monitoring real-time sensor feed...\n")

        while True:
            if ser.in_waiting > 0:
                raw_line = ser.readline()
                decoded_line = raw_line.decode('utf-8', errors='ignore').strip()

                if decoded_line:
                    distance = parse_distance(decoded_line)

                    if distance is not None:
                        print(f"Distance: {distance:.1f} cm")

                        zone = get_active_zone(distance)

                        if zone is not None:
                            print(f"\n⚠️ Box Detected in Zone {zone}! ({distance:.1f} cm)")

                            delay_seconds = int(DELAY_BEFORE_MOVE_SEC)
                            print(f"Holding for {DELAY_BEFORE_MOVE_SEC} seconds before initiating pick...")

                            if delay_seconds > 0:
                                for remaining in range(delay_seconds, 0, -1):
                                    print(f"  Starting in {remaining}...")
                                    time.sleep(1)
                            else:
                                time.sleep(DELAY_BEFORE_MOVE_SEC)

                            # Trigger appropriate arm based on zone
                            if zone == 1:
                                execute_arm1_pick_and_place(arm1)
                            elif zone == 2:
                                execute_arm2_pick_and_place(arm2)

                            # Flush all accumulated serial data from buffer
                            ser.reset_input_buffer()
                            time.sleep(0.5)
                            ser.reset_input_buffer()

                            print("Resuming sensor monitoring...")

            # Tiny sleep to keep CPU cool
            time.sleep(0.001)

    except serial.SerialException as e:
        print(f"\n[Error] Could not open {PORT}: {e}")
    except KeyboardInterrupt:
        print("\nProgram stopped manually.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port safely disconnected.")


if __name__ == "__main__":
    main()
# import sys
# import time
# import serial
# import xarm
#
# # CONFIGURATION
# PORT = "COM10"
# BAUD_RATE = 9600
#
# # Target Zone 1 -> Arm 1 Only
# ZONE1_MIN_CM = 15.0
# ZONE1_MAX_CM = 25.0
#
# # Target Zone 2 -> Arm 2 Only
# ZONE2_MIN_CM = 1.0
# ZONE2_MAX_CM = 14.0
#
#
# DELAY_BEFORE_MOVE_SEC = 0.5  # Delay before initiating pick
#
# # Calibrated claw positions
# CLAW_OPEN = 100
# CLAW_CLOSE = 1000
#
#
# def initialize_arms():
#     """Initializes arm controllers and returns their instances."""
#     print("Connecting to xArm controllers...")
#     arm1 = xarm.Controller("USB1")  # Front-Right Arm
#     arm2 = xarm.Controller("USB0")  # Back-Left Arm
#     return arm1, arm2
#
#
# def reset_arm1(arm1):
#     """Resets Arm 1 to its ready position."""
#     print("Setting Arm 1 to Ready Position...")
#     arm1.setPosition(1, CLAW_OPEN)
#     arm1.setPosition(2, 500)
#     arm1.setPosition(3, 330)
#     arm1.setPosition(4, 780)
#     arm1.setPosition(5, 750)
#     arm1.setPosition(6, 860)
#
#
# def reset_arm2(arm2):
#     """Resets Arm 2 to its ready position."""
#     print("Setting Arm 2 to Ready Position...")
#     arm2.setPosition(1, CLAW_OPEN)
#     arm2.setPosition(2, 900)
#     arm2.setPosition(3, 480)
#     arm2.setPosition(4, 400)
#     arm2.setPosition(5, 250)
#     arm2.setPosition(6, 100)
#
#
# def reset_to_master_set_position(arm1, arm2):
#     """Moves both arms back to their calibrated ready position."""
#     reset_arm1(arm1)
#     reset_arm2(arm2)
#     time.sleep(2.0)
#
#
# def execute_arm1_pick_and_place(arm1):
#     """Executes pick and place sequence using ONLY Arm 1."""
#     print("\n--- Starting Arm 1 Pick and Place Sequence ---")
#
#     # Grab Box
#     print("Arm 1: Closing gripper...")
#     arm1.setPosition(1, CLAW_CLOSE)
#     time.sleep(1.5)
#
#     # Lift
#     print("Arm 1: Lifting...")
#     arm1.setPosition(3, 10)
#     arm1.setPosition(4, 600)
#     arm1.setPosition(5, 600)
#     time.sleep(1.0)
#
#     # Pivot
#     print("Arm 1: Pivoting...")
#     arm1.setPosition(5, 550)
#     arm1.setPosition(6, 500)
#     time.sleep(1.0)
#
#     # Drop Box
#     print("Arm 1: Releasing object...")
#     arm1.setPosition(1, CLAW_OPEN)
#     time.sleep(2.0)
#
#     # Reset to Ready Position
#     print("Arm 1: Returning to ready position...")
#     arm1.setPosition(6, 880)
#     time.sleep(1.0)
#     arm1.setPosition(2, 500)
#     arm1.setPosition(3, 290)
#     arm1.setPosition(4, 800)
#     time.sleep(1.0)
#     arm1.setPosition(5, 750)
#     time.sleep(1.0)
#     print("--- Arm 1 Pick and Place Complete ---\n")
#
#
# def execute_arm2_pick_and_place(arm2):
#     """Executes pick and place sequence using ONLY Arm 2."""
#     print("\n--- Starting Arm 2 Pick and Place Sequence ---")
#
#     # Grab Box
#     print("Arm 2: Closing gripper...")
#     arm2.setPosition(1, CLAW_CLOSE)
#     time.sleep(1.5)
#
#     # Lift
#     print("Arm 2: Lifting...")
#     arm2.setPosition(3, 800)
#     arm2.setPosition(4, 600)
#     arm2.setPosition(5, 430)
#     time.sleep(1.0)
#
#     # Pivot
#     print("Arm 2: Pivoting...")
#     arm2.setPosition(5, 500)
#     arm2.setPosition(6, 500)
#     time.sleep(1.0)
#
#     # Drop Box
#     print("Arm 2: Releasing object...")
#     arm2.setPosition(1, CLAW_OPEN)
#     time.sleep(2.0)
#
#     # Reset to Ready Position
#     print("Arm 2: Returning to ready position...")
#     arm2.setPosition(6, 100)
#     time.sleep(1.0)
#     arm2.setPosition(2, 900)
#     arm2.setPosition(3, 480)
#     arm2.setPosition(4, 400)
#     time.sleep(1.0)
#     arm2.setPosition(5, 260)
#     arm2.setPosition(6, 100)
#     time.sleep(1.0)
#     print("--- Arm 2 Pick and Place Complete ---\n")
#
#
# def parse_distance(line):
#     """Safely extracts distance float value from incoming string lines."""
#     try:
#         if "Distance:" in line:
#             parts = line.split("Distance:")
#             val_part = parts[1].strip().split()[0]
#             return float(val_part)
#     except (IndexError, ValueError):
#         pass
#     return None
#
#
# def get_active_zone(distance):
#     """Determines which zone (1 or 2) the distance falls into."""
#     if ZONE1_MIN_CM <= distance <= ZONE1_MAX_CM:
#         return 1
#     elif ZONE2_MIN_CM <= distance <= ZONE2_MAX_CM:
#         return 2
#     return None
#
#
# def main():
#     print("--- High-Frequency Python Sensor Listener with Selective xArm Automation ---")
#     print(f"Zone 1 ({ZONE1_MIN_CM}-{ZONE1_MAX_CM} cm) -> Triggers Arm 1")
#     print(f"Zone 2 ({ZONE2_MIN_CM}-{ZONE2_MAX_CM} cm) -> Triggers Arm 2")
#
#     # Initialize arms
#     arm1, arm2 = initialize_arms()
#     reset_to_master_set_position(arm1, arm2)
#
#     print(f"Connecting to sensor on {PORT}...")
#
#     try:
#         ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
#         time.sleep(2)  # Allow Arduino reboot window
#
#         ser.reset_input_buffer()
#         print("Connected! Monitoring real-time sensor feed...\n")
#
#         while True:
#             if ser.in_waiting > 0:
#                 raw_line = ser.readline()
#                 decoded_line = raw_line.decode('utf-8', errors='ignore').strip()
#
#                 if decoded_line:
#                     distance = parse_distance(decoded_line)
#
#                     if distance is not None:
#                         print(f"Distance: {distance:.1f} cm")
#
#                         zone = get_active_zone(distance)
#
#                         if zone is not None:
#                             print(f"\n⚠️ Box Detected in Zone {zone}! ({distance:.1f} cm)")
#
#                             delay_seconds = int(DELAY_BEFORE_MOVE_SEC)
#                             print(f"Holding for {DELAY_BEFORE_MOVE_SEC} seconds before initiating pick...")
#
#                             if delay_seconds > 0:
#                                 for remaining in range(delay_seconds, 0, -1):
#                                     print(f"  Starting in {remaining}...")
#                                     time.sleep(1)
#                             else:
#                                 time.sleep(DELAY_BEFORE_MOVE_SEC)
#
#                             # Trigger appropriate arm based on zone
#                             if zone == 1:
#                                 execute_arm1_pick_and_place(arm1)
#                             elif zone == 2:
#                                 execute_arm2_pick_and_place(arm2)
#
#                             # Flush all accumulated serial data from buffer
#                             ser.reset_input_buffer()
#                             time.sleep(0.5)
#                             ser.reset_input_buffer()
#
#                             print("Resuming sensor monitoring...")
#
#             # Tiny sleep to keep CPU cool
#             time.sleep(0.001)
#
#     except serial.SerialException as e:
#         print(f"\n[Error] Could not open {PORT}: {e}")
#     except KeyboardInterrupt:
#         print("\nProgram stopped manually.")
#     finally:
#         if 'ser' in locals() and ser.is_open:
#             ser.close()
#             print("Serial port safely disconnected.")
#
#
# if __name__ == "__main__":
#     main()