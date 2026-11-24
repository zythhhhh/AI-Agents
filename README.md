# FAMS — Manufacturing Robotics

Research platform using three Hiwonder xArm 1S arms, a conveyor, and a planned Isaac Sim digital twin.

## Development

Legacy experiment folders are preserved. New diagnostics and calibration live in `robot_control/`. Use Python 3.11 and install dependencies with `python -m pip install xarm hidapi pyserial`.

## First test

Connect only one physically labeled arm with its verified power supply. Clear its workspace and close the vendor control software before running:

```
python robot_control/read_positions.py
```

Record measured positions and the physical pose. Missing or failed readings are not valid calibration values. No automatic motion is enabled in the new scripts. Do not assume position 500 is a safe home pose.

## Roadmap

1. Verify each arm and establish consistent physical reference poses and safe joint limits.
2. Verify USB identity and independent multi-arm control.
3. Develop deterministic LEGO manipulation and assembly.
4. Develop and validate the Isaac Sim digital twin; investigate learning methods after baseline testing.

See ENGINEERING_LOG.md for progress.

## Hardware identification

- Arm 1: four reported active servos (3–6), no wrist rotation or gripper; upright reference adjusted by teammate. Photo IMG_4453.jpeg.
- Arm 2: large serrated claw, aluminum frame. Photo IMG_4454.jpeg.
- Arm 3: smaller precision gripper, black platform. Photo IMG_4455.jpeg.

Photo filenames refer to conversation uploads, not repository files. Servo communication, reference positions and limits remain to be verified.
