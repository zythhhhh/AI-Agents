"""Single-arm diagnostics; no motion on import."""
import json
from pathlib import Path
import xarm

CONFIG = Path(__file__).with_name("calibration.json")

def load_calibration():
    return json.loads(CONFIG.read_text(encoding="utf-8"))

def connect():
    return xarm.Controller("USB", debug=False)

def read_positions(arm):
    positions = {}
    for servo in range(1, 7):
        try:
            positions[str(servo)] = arm.getPosition(servo)
        except Exception as exc:
            positions[str(servo)] = None
            print(f"Servo {servo}: read failed: {exc}")
    return positions
