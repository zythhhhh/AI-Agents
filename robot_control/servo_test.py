"""Preview a small joint test; motion disabled pending physical calibration."""
from arm_controller import connect, read_positions

def main():
    arm = connect()
    positions = read_positions(arm)
    print("Current positions:", positions)
    servo = input("Servo ID to inspect (1-6): ").strip()
    if servo not in positions:
        raise SystemExit("Invalid servo ID")
    print("Current position:", positions[servo])
    print("Motion disabled until joint mapping and safe limits are verified.")

if __name__ == "__main__":
    main()
