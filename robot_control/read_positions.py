"""Read one connected arm without commanding motion."""
from arm_controller import connect, read_positions

def main():
    arm = connect()
    print("Connected to xArm")
    print("Voltage:", arm.getBatteryVoltage())
    for servo, position in read_positions(arm).items():
        print(f"Servo {servo}: {position}")

if __name__ == "__main__":
    main()
