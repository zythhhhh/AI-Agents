"""Single-arm diagnostics; no motion on import."""
import json
import time
from pathlib import Path
import xarm

CONFIG = Path(__file__).with_name("calibration.json")

def load_calibration():
    return json.loads(CONFIG.read_text(encoding="utf-8"))

def connect():
    return xarm.Controller("USB", debug=False)

def read_positions(arm, servos=range(1, 7)):
    positions = {}
    for servo in servos:
        try:
            positions[str(servo)] = arm.getPosition(servo)
        except Exception as exc:
            positions[str(servo)] = None
            print(f"Servo {servo}: read failed: {exc}")
        finally:
            time.sleep(0.5)
    return positions
