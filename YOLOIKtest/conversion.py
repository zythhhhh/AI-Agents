import cv2
import numpy as np


pixel_points = np.array([
    [595,  40],   # TL
    [604, 437],   # TR
    [ 78,  33],   # BL
    [ 71, 437],   # BR
], dtype=np.float32)

robot_points = np.array([
    [0.4350,  0.0291],   # TL
    [0.4465, -0.1305],   # TR
    [0.1409,  0.0141],   # BL
    [0.1524, -0.1455],   # BR
], dtype=np.float32)


# Calculate transformation
H, _ = cv2.findHomography(
    pixel_points,
    robot_points
)

print("Homography matrix:")
print(H)


def pixel_to_robot(u, v):

    point = np.array(
        [[[u, v]]],
        dtype=np.float32
    )

    result = cv2.perspectiveTransform(
        point,
        H
    )[0][0]

    return float(result[0]), float(result[1])

while True:

    u = float(input("\nPixel X (u): "))
    v = float(input("Pixel Y (v): "))

    x, y = pixel_to_robot(u, v)

    print(
        f"Robot position: "
        f"X = {x:.4f} m, "
        f"Y = {y:.4f} m"
    )
# ============================================================
# TEST A PIXEL
# ============================================================

u = 300
v = 200

x, y = pixel_to_robot(u, v)

print("\nInput pixel:")
print(f"u = {u}, v = {v}")

print("\nConverted robot position:")
print(f"X = {x:.4f} m")
print(f"Y = {y:.4f} m")