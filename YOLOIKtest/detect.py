import cv2
from ultralytics import YOLOWorld


model = YOLOWorld("yolov8s-worldv2.pt")

classes = [
    "red cube",
    "blue cube",
    "yellow cube",
    "green cube",
]

model.set_classes(classes)

# Change this if your camera isn't index 0
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    raise RuntimeError("Could not open camera")


target_color = input("Which cube? red / blue / yellow: ").strip().lower()
target_label = f"{target_color} cube"


while True:
    ok, frame = cap.read()

    if not ok:
        continue

    # Run YOLO
    results = model.predict(
        frame,
        conf=0.20,
        verbose=False,
    )

    result = results[0]

    detections = []

    for box in result.boxes:

        # Bounding box corners
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

        confidence = float(box.conf[0])
        class_id = int(box.cls[0])

        label = result.names[class_id]

        # Center of bounding box
        u = (x1 + x2) / 2
        v = (y1 + y2) / 2

        detections.append({
            "label": label,
            "confidence": confidence,
            "u": float(u),
            "v": float(v),
            "bbox": (int(x1), int(y1), int(x2), int(y2)),
        })

        # Draw visualization
        cv2.rectangle(
            frame,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            (0, 255, 0),
            2,
        )

        cv2.circle(
            frame,
            (int(u), int(v)),
            5,
            (0, 0, 255),
            -1,
        )

        cv2.putText(
            frame,
            f"{label} {confidence:.2f}",
            (int(x1), int(y1) - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    # Find requested cube
    target_detections = [
        d for d in detections
        if d["label"] == target_label
    ]

    if target_detections:

        # Use highest-confidence detection
        target = max(
            target_detections,
            key=lambda x: x["confidence"],
        )

        print(
            f"{target['label']}: "
            f"pixel=({target['u']:.1f}, {target['v']:.1f}), "
            f"confidence={target['confidence']:.2f}"
        )

    cv2.imshow("YOLO Cube Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()