import cv2
import numpy as np
import serial
import time
from ultralytics import YOLO
import threading
import subprocess
# Load a pretrained YOLO model (replace with your trained model path)
model = YOLO("best.pt")

# Initialize serial connection to Arduino
arduino_right = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=0.1)
arduino_left = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=0.1)
time.sleep(2)  # Wait for Arduino to initialize

# Flag to indicate if the robot is currently processing an object
processing_right = False
processing_left = False

def send_to_arduino_right(mapped_x, mapped_y):
    global processing_right
    data = f"#{mapped_x},{mapped_y}\n"
    arduino_right.write(data.encode())
    print(f"Sent to Arduino: {data}")
    processing_right = True  # Set processing flag

def wait_for_feedback_right():
    global processing_right
    while True:
        if arduino_right.in_waiting > 0:
            feedback = arduino_right.readline().decode('utf-8').rstrip()
            print(feedback)
            
            # Memeriksa apakah feedback terdiri dari tiga integer yang dipisahkan oleh koma
            if feedback.count(',') == 2:
                try:
                    # Memisahkan feedback menjadi tiga nilai integer
                    ax1, ax2, ax3 = map(int, feedback.split(','))
                    ax1=arduino_map(ax1, 0, 180, 0, 180)
                    ax2=arduino_map(ax2, 86, 0, 0, 90)
                    ax3=arduino_map(ax3, 30, 114, 0, 90)
                    
                    # Menjalankan perintah ROS 2 dengan nilai-nilai yang diparsing
                    process = subprocess.Popen([
                        'ros2', 'launch', 'arm_robot', 'control.launch.py', 
                        f'ax1:={ax1}', f'ax2:={ax2}', f'ax3:={ax3}'
                    ])
                    print(f"ROS 2 launch command executed with ax1={ax1}, ax2={ax2}, ax3={ax3}.")
                    
                    # Tunggu sampai proses selesai, jika diperlukan
                    process.wait()
                
                except ValueError:
                    print("Invalid feedback format. Could not parse integers.")
            
            # Memeriksa apakah feedback adalah "A"
            elif feedback == "A":
                print("Received feedback from Arduino:", feedback)
                process = subprocess.Popen([
                        'ros2', 'launch', 'arm_robot', 'control.launch.py', 
                        f'ax1:={0}', f'ax2:={90}', f'ax3:={0}'
                    ])
                print(f"ROS 2 launch command executed home")
                    
                    # Tunggu sampai proses selesai, jika diperlukan
                process.wait()
                processing_right = False  # Reset processing flag
                break
        
        time.sleep(0.1)  
def send_data_with_feedback_right(mapped_x, mapped_y):
    send_to_arduino_right(mapped_x, mapped_y)
    wait_for_feedback_right()


def send_to_arduino_left(mapped_x, mapped_y):
    global processing_left
    data = f"#{mapped_x},{mapped_y}\n"
    arduino_left.write(data.encode())
    print(f"Sent to Arduino: {data}")
    processing_left = True  # Set processing flag

def wait_for_feedback_left():
    global processing_left
    while True:
        if arduino_left.in_waiting > 0:
            feedback = arduino_left.readline().decode('utf-8').rstrip()
            if feedback == "A":
                print("Received feedback from Arduino:", feedback)
                processing_left = False  # Reset processing flag
                break
        time.sleep(0.1)  # Small delay to avoid busy waiting

def send_data_with_feedback_left(mapped_x, mapped_y):
    send_to_arduino_left(mapped_x, mapped_y)
    wait_for_feedback_left()

def detect_green_border(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_green = np.array([161, 69, 45])
    upper_green = np.array([188, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    return mask

def get_contours(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def get_largest_contour(contours):
    largest_contour = max(contours, key=cv2.contourArea)
    return largest_contour

def get_four_points_from_contour(contour):
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    return approx

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def four_point_transform(image, pts, width, height):
    rect = order_points(pts)
    dst = np.array([
        [-130, -90],
        [width + 150, -90],
        [width + 130, height + 80],
        [-140, height + 90]], dtype="float32")

    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (width, height))
    return warped

def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

# Conversion of paper size to pixels with DPI 300
DPI = 300
width_cm = 60
height_cm = 40
width_px = 1080
height_px = 720

cap = cv2.VideoCapture("http://192.168.100.82:8080/video")

last_mapped_x = None
last_mapped_y = None
threshold = 10  # Define a threshold for significant changes
process = subprocess.Popen([
                        'ros2', 'launch', 'arm_robot', 'control.launch.py', 
                        f'ax1:={0}', f'ax2:={90}', f'ax3:={0}'
                    ])
print(f"ROS 2 launch command executed home")
                    
                    # Tunggu sampai proses selesai, jika diperlukan
process.wait()
while True:
    ret, frame = cap.read()
    print("oke")
    if not ret:
        break
    print("oke1")
    mask = detect_green_border(frame)
    contours = get_contours(mask)
    if contours:
        largest_contour = get_largest_contour(contours)
        approx = get_four_points_from_contour(largest_contour)
        if len(approx) == 4:
            warped = four_point_transform(frame, approx.reshape(4, 2), width_px, height_px)
            results = model(warped)
            height, width, _ = warped.shape

            # Filter out the bounding box with the highest confidence for each object
            detected_objects = {}
            for r in results:
                for box in r.boxes:
                    class_index = int(box.cls[0])
                    confidence = box.conf[0]

                    if class_index not in detected_objects or confidence > detected_objects[class_index][0]:
                        detected_objects[class_index] = (confidence, box)

            # Process and send the bounding box data
            for _, (confidence, box) in detected_objects.items():
                if not processing_right:  # Only process if not currently processing another object
                    x1, y1, x2, y2 = box.xyxy[0]

                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    if center_x > int(width / 2):
                        center_x_real = center_x - (int(width / 2))
                        mapped_value_x = arduino_map(center_y, 0, height, 200, 0)
                        mapped_value_y = arduino_map(center_x_real, 0, width / 2, 0, 50)

                        # Only send data if the change is significant
                        if (last_mapped_x is None or 
                            abs(mapped_value_x - last_mapped_x) > threshold or 
                            abs(mapped_value_y - last_mapped_y) > threshold):

                            # Update last known positions
                            last_mapped_x = mapped_value_x
                            last_mapped_y = mapped_value_y

                            # Use threading to send data without blocking
                            threading.Thread(target=send_data_with_feedback_right, args=(mapped_value_x, mapped_value_y)).start()

                        cv2.rectangle(warped, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.circle(warped, (center_x, center_y), 5, (0, 0, 255), -1)
                        cv2.putText(warped, f'({int(mapped_value_x)}, {int(mapped_value_y)})', (center_x, center_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                if not processing_left:  # Only process if not currently processing another object
                    x1, y1, x2, y2 = box.xyxy[0]

                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    if center_x < int(width / 2):
                        center_x_real = center_x - (int(width / 2))
                        mapped_value_x = arduino_map(center_y, height, 0, 0, 200)
                        mapped_value_y = arduino_map(center_x, width / 2, 0, 0, 50)

                        # Only send data if the change is significant
                        if (last_mapped_x is None or 
                            abs(mapped_value_x - last_mapped_x) > threshold or 
                            abs(mapped_value_y - last_mapped_y) > threshold):

                            # Update last known positions
                            last_mapped_x = mapped_value_x
                            last_mapped_y = mapped_value_y

                            # Use threading to send data without blocking
                            threading.Thread(target=send_data_with_feedback_left, args=(mapped_value_x, mapped_value_y)).start()

                        cv2.rectangle(warped, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.circle(warped, (center_x, center_y), 5, (0, 0, 255), -1)
                        cv2.putText(warped, f'({int(mapped_value_x)}, {int(mapped_value_y)})', (center_x, center_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            cv2.imshow("Warped", warped)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
