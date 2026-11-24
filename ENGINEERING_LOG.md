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

## Arm 1 initial motion tests

Updated diagnostic succeeded: 3=553, 4=499, 5=517, 6=501. Operator confirmed unchanged upright reference pose. Each servo was individually commanded +20 units over 2000 ms, then back to reference. All returned visually. Increasing ID 6 rotated base counterclockwise, ID 5 tilted forward (operator reported), ID 4 moved toward operator, ID 3 away. Joint mappings except base and robot-frame directions remain provisional. Final position readback pending. No safe limits or home trajectory established.
