"""Read one connected arm without commanding motion."""
import argparse
from arm_controller import connect, read_positions, load_calibration

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=["arm1", "arm2", "arm3"], required=True)
    args = parser.parse_args()
    servos = load_calibration()[args.arm]["active_servos"]
    if not servos:
        raise SystemExit("Active servo IDs not verified for this arm.")
    arm = connect()
    print("Connected to xArm:", args.arm)
    for servo, position in read_positions(arm, servos).items():
        print(f"Servo {servo}: {position}")

if __name__ == "__main__":
    main()
