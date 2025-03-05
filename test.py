import cv2
import serial
import time
import threading
from queue import Queue
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO(r"C:\Users\Lachhu\Documents\chakka\best.pt")

# Set up serial communication with ESP32
ser = serial.Serial('COM8', 115200, timeout=1)
time.sleep(2)  # Allow time for serial connection

# ESP32-CAM stream URL
stream_url = "http://192.168.253.1:81/stream"

# Frame queue
frame_queue = Queue(maxsize=2)
running = True

# Frame reading thread
def read_frames():
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        print("Error: Could not open ESP32-CAM stream.")
        return
    
    while running:
        ret, frame = cap.read()
        if ret:
            if not frame_queue.full():
                frame_queue.put(frame)
        else:
            print("Failed to read frame from ESP32-CAM.")
            time.sleep(0.1)  # Small delay if frame read fails
    cap.release()

# Start frame reading thread
threading.Thread(target=read_frames, daemon=True).start()

while True:
    if not frame_queue.empty():
        frame = frame_queue.get()

        # Run YOLO detection with confidence threshold
        results = model(frame, conf=0.4)

        detected_class = "None"
        detection_found = False  # Flag to track valid detections

        # Iterate over detections
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            class_name = model.names[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Check for Tiger and Elephant
            if class_name.lower() == "tiger":
                color = (0, 0, 255)
                detected_class = "Tiger"
                ser.write(b'R')
                detection_found = True

            elif class_name.lower() == "elephant":
                color = (0, 255, 255)
                detected_class = "Elephant"
                ser.write(b'Y')
                detection_found = True

            else:
                color = (255, 255, 255)

            # Draw bounding box and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, class_name, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Send 'N' if no target detected
        if not detection_found:
            ser.write(b'N')

        # Display frame
        cv2.putText(frame, f"Detected: {detected_class}", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("YOLOv8 Detection", frame)

    # Add delay to slow down detection rate
    time.sleep(0.1)

    # Exit on pressing 'q'
    if cv2.waitKey(10) & 0xFF == ord('q'):
        running = False
        break

# Release resources
cv2.destroyAllWindows()
ser.close()