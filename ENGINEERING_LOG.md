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

## Arm 1 forward-facing neutral alignment

Initial return readback: 554, 500, 517, 505 (IDs 3–6); absolute errors 1, 1, 0, 4. Base incrementally commanded to 875 for operator-defined forward, approximately 90 degrees counterclockwise from initial pose. Unexpected wrist/elbow shifts during base rotation remain unexplained. After visual elbow and wrist adjustments, operator confirmed alignment. Final measured neutral: 3=578, 4=513, 5=517, 6=872. Offsets relative to logical 500: +78, +13, +17, +372. These are reference data only, not implemented motion conversion. Repeatability, safe limits and home trajectory remain unverified.

## Arm 1 preliminary calibration completion

At operator request, shortened testing for this session only. Tested command ranges: wrist 478–678 (increasing backward), elbow 413–613 (increasing forward), shoulder 317–617 (increasing backward), base 501–875 (increasing counterclockwise). Directions refer to newly defined forward orientation. Shoulder command 467 initially failed to move; retry with debug succeeded, readback 469; cause unknown. At command 367 readback 368, other joints stable. Range endpoints generally visually confirmed, not all measured. Final neutral readback IDs 3–6: 576, 513, 517, 872; absolute errors 2, 0, 0, 0. Preliminary calibration complete. Safe limits, combined motions, payload testing and repeatability remain pending; home and limits fields remain empty.

## 2026-11-24 — Arm 2 preliminary calibration

Using the same controller individually, all six IDs responded: 461, 900, 916, 278, 415, 132 (IDs 1–6). Operator performed physical alignment and shortened bidirectional tests. Shoulder initially suspected nonresponsive; terminal history showed no Arm 2 shoulder command yet, subsequent movement confirmed. Temporary weighted base and suspected screw/bracket alignment issue require inspection and secure mounting before further loaded/coordinated testing.

| ID | Joint | Reference | Increasing direction | Command range | Final readback |
|---|---|---:|---|---|---:|
| 1 | Gripper | 288 open / 388 closed | closes | 288–438 | 381 |
| 2 | Wrist rotation | 849 | left (operator view) | 799–899 | 850 |
| 3 | Wrist pitch | 510 | forward | 460–560 | 515 |
| 4 | Elbow | 498 | backward | 378–548 | 499 |
| 5 | Shoulder | 503 | backward | 453–543 | 497 |
| 6 | Base | 124 | CCW viewed above | 124–174 | 124 |

Single final readback errors relative to targets (closed gripper): -7, +1, +5, +1, -6, 0; mean absolute error 3.33 units, max 7. Command ranges are not validated limits; endpoints not all independently measured. Gripper 438 was exploratory. Neutral offsets stored relative to 500 for IDs 2–6 only; no conversion implemented. No safe home trajectory, payload, repeatability or simultaneous control validation. Reference photo IMG_4463.jpeg not committed.

## 2026-11-24 — Arm 3 preliminary calibration

Operator manually aligned the arm before connection. All six IDs responded. Individual bidirectional tests completed in order 6, 5, 4, 3, 2, 1 using the shared controller.

| ID | Joint | Reference | Increasing direction | Command range | Final readback |
|---|---|---:|---|---|---:|
| 1 | Gripper | 437 open / 637 closed | closes | 437–637 | 623 |
| 2 | Wrist rotation | 526 | left (operator view) | 476–576 | 527 |
| 3 | Wrist pitch | 509 | forward | 459–559 | 511 |
| 4 | Elbow | 491 | backward | 441–541 | 491 |
| 5 | Shoulder | 519 | forward | 469–569 | 521 |
| 6 | Base | 870 | CCW viewed above | 820–920 | 873 |

Final errors: -14, +1, +2, 0, +2, +3; mean absolute error 3.67 units, maximum 14. Gripper return discrepancy requires investigation; do not increase closing force to compensate. Shoulder direction differs from Arms 1 and 2. All three arms now preliminarily calibrated individually (16 active servos total). No simultaneous controller validation, safe home, validated limits, payload or repeatability tests. Photo IMG_4464.jpeg documents Arms 1 (left) and 2 (right) facing designated forward; not committed. Offsets are data only.
