"""Preview recorded home pose without moving hardware."""
from arm_controller import load_calibration

def main():
    name = input("Arm (arm1/arm2/arm3): ").strip()
    config = load_calibration().get(name)
    if not config or not config["home"]:
        raise SystemExit("Home pose not calibrated.")
    print("Recorded home:", config["home"])
    print("Motion disabled until a safe home trajectory is verified.")

if __name__ == "__main__":
    main()
