import cv2
from ultralytics import YOLO

model = YOLO("models/yolo26n.pt")

camera = cv2.VideoCapture(0)

while camera.isOpened():
    received, frame = camera.read()

    if not received:
        print("Error: Receiving frame")
        break

    results = model(frame)
    result = results[0]

    for box in result.boxes:
        class_id = int(box.cls[0])
        confidence = float(box.conf[0])

        object_detected = result.names[class_id]

        print(f"{object_detected}: {confidence:.2f}")

    
    annotated_frame = results[0].plot()

    cv2.imshow("Live Video Feed", annotated_frame)
    key = cv2.waitKey(1)

    if key == ord('q'):
        print("Bai Bai!")
        break

camera.release()
cv2.destroyAllWindows()
