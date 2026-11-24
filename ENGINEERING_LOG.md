# Engineering log

## 2026-11-24 — Calibration preparation

Reported progress: conveyor tested; preliminary single-arm USB tests performed; one arm reportedly lacks a gripper. Exact arm identity and joint calibration remain unverified.

New read-only diagnostics and calibration schema prepared. No hardware validation of these scripts has been performed.

Next: label arms, identify missing gripper, read positions individually, document reference poses, verify joint mapping and safe limits, then test repeatable home trajectories and multi-arm USB addressing.

### Measurements

| Arm | Servo 1 | Servo 2 | Servo 3 | Servo 4 | Servo 5 | Servo 6 | Physical pose / notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| arm1 | N/A | N/A | 553 | 499 | 516 | 501 | Initial readings; pose not yet verified |
| arm2 | | | | | | | |
| arm3 | | | | | | | |

## Hardware identification

- Arm 1: four reported active servos (3–6), no wrist rotation or gripper; upright reference adjusted by teammate. Photo IMG_4453.jpeg.
- Arm 2: large serrated claw, aluminum frame. Photo IMG_4454.jpeg.
- Arm 3: smaller precision gripper, black platform. Photo IMG_4455.jpeg.

Photo filenames refer to conversation uploads, not repository files. Servo communication, reference positions and limits remain to be verified.

## Arm 1 communication test — 2026-11-24

Reported voltage 7.753 V. Initial reads failed with list index out of range. Individual reads of IDs 3–6 with 0.5 s spacing succeeded twice, with debug enabled and disabled. Exact cause of initial failure unconfirmed. Updated read-only diagnostic requires explicit arm selection and omits voltage polling to match successful test. Updated script still needs hardware testing. Home, neutral, limits and directions remain uncalibrated.
