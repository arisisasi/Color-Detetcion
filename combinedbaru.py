import cv2
import numpy as np
import serial
import time
from ultralytics import YOLO
import threading

# Load a pretrained YOLO model (replace with your trained model path)
model = YOLO("best.pt")

arduino_right = None
arduino_left = None
port_right = 'COM9'
port_left = 'COM10'
baudrate = 9600
def find_min_x_pair(array):
    # Inisialisasi pasangan dengan nilai x terkecil
    min_pair = array[0]
    
    # Iterasi melalui setiap pasangan [x, y]
    for pair in array:
        if pair[0] < min_pair[0]:
            min_pair = pair
    
    return min_pair

def find_max_x_pair(array):
    # Inisialisasi pasangan dengan nilai x terbesar
    max_pair = array[0]
    
    # Iterasi melalui setiap pasangan [x, y]
    for pair in array:
        if pair[0] > max_pair[0]:
            max_pair = pair
    
    return max_pair
def connect_arduino(port):
    try:
        arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        print(f"Arduino connected on {port}.")
        return arduino
    except serial.SerialException:
        print(f"Arduino not connected on {port}. Retrying...")
        return None

time.sleep(2)  # Wait for Arduino to initialize
arduino_right = connect_arduino(port_right)
arduino_left = connect_arduino(port_left)
# Flag to indicate if the robot is currently processing an object
processing_right = False
processing_left = False

import serial.tools.list_ports

def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "Arduino" in port.description:  # Adjust this condition as necessary
            return port.device
    return None

def reconnect_arduino_right():
    global arduino_right,port_right
    while arduino_right is None:
        print("reconect")
        # port_right = port_right
        if port_right is not None:
            arduino_right = connect_arduino(port_right)
            if arduino_right is not None:
                print(f"Successfully reconnected to Arduino Right on {port_right}!")
        time.sleep(5)  # Retry every 3 seconds

def reconnect_arduino_left():
    global arduino_left,port_left
    while arduino_left is None:
        # port_left = find_arduino_port()
        if port_left is not None:
            arduino_left = connect_arduino(port_left)
            if arduino_left is not None:
                print(f"Successfully reconnected to Arduino Left on {port_left}!")
        time.sleep(5)  # Retry every 3 seconds


def send_to_arduino_right(mapped_x, mapped_y):
    global processing_right,arduino_right
    if arduino_right is not None:
        try:
            data = f"#{mapped_x},{mapped_y}\n"
            arduino_right.write(data.encode())
            print(f"Sent to Arduino Right: {data}")
            processing_right = True  # Set processing flag
        except serial.SerialException:
            print("Error: Arduino Right disconnected!")
            arduino_right = None
            processing_right = False
            threading.Thread(target=reconnect_arduino_right).start()  # Attempt to reconnect
    else:
        print("Arduino Right is not connected.")

def wait_for_feedback_right():
    global processing_right,arduino_right
    if arduino_right is not None:
        try:
            while True:
                if arduino_right.in_waiting > 0:
                    feedback = arduino_right.readline().decode('utf-8').rstrip()
                    if feedback == "A":
                        print("Received feedback from Arduino Right:", feedback)
                        processing_right = False  # Reset processing flag
                        break
                time.sleep(0.1)  # Small delay to avoid busy waiting
        except serial.SerialException:
            print("Error: Arduino Right disconnected!")
            arduino_right = None
            processing_right = False
            threading.Thread(target=reconnect_arduino_right).start()  # Attempt to reconnect
    else:
        print("Arduino Right is not connected.")

def send_data_with_feedback_right(mapped_x, mapped_y):
    print('kirimkanan')
    send_to_arduino_right(mapped_x, mapped_y)
    wait_for_feedback_right()


def send_to_arduino_left(mapped_x, mapped_y):
    global processing_left,arduino_left
    if arduino_left is not None:
        try:
            data = f"#{mapped_x},{mapped_y}\n"
            arduino_left.write(data.encode())
            print(f"Sent to Arduino Left: {data}")
            processing_left = True  # Set processing flag
        except serial.SerialException:
            print("Error: Arduino Left disconnected!")
            arduino_left = None
            processing_left = False
            threading.Thread(target=reconnect_arduino_left).start()  # Attempt to reconnect
    else:
        print("Arduino Left is not connected.")

def wait_for_feedback_left():
    global processing_left,arduino_left
    if arduino_left is not None:
        try:
            while True:
                if arduino_left.in_waiting > 0:
                    feedback = arduino_left.readline().decode('utf-8').rstrip()
                    if feedback == "A":
                        print("Received feedback from Arduino Left:", feedback)
                        processing_left = False  # Reset processing flag
                        break
                time.sleep(0.1)  # Small delay to avoid busy waiting
        except serial.SerialException:
            print("Error: Arduino Left disconnected!")
            arduino_left = None
            processing_left = False
            threading.Thread(target=reconnect_arduino_left).start()  # Attempt to reconnect
    else:
        print("Arduino Left is not connected.")

def send_data_with_feedback_left(mapped_x, mapped_y):
    print("kirimkiri")
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
last_mapped_x = None
last_mapped_y = None
def kanan(warped):
    global last_mapped_x,last_mapped_y
    height, width, _ = warped.shape
    if not processing_right:  # Only process if not currently processing another object
        x1, y1, x2, y2 = box.xyxy[0]

        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        if(center_x>=width/2):

        # if center_x > int(width / 2):
        #     center_x_real = center_x - (int(width / 2))
            mapped_value_x = arduino_map(center_y, 0, height, 0, 200)
            mapped_value_y = arduino_map(center_x, 280, width , 75, 0)

        #     # Only send data if the change is significant
            if (last_mapped_x is None or 
                abs(mapped_value_x - last_mapped_x) > threshold or 
                abs(mapped_value_y - last_mapped_y) > threshold):

                # Update last known positions
                last_mapped_x = mapped_value_x
                last_mapped_y = mapped_value_y

                # Use threading to send data without blocking
                threading.Thread(target=send_data_with_feedback_right, args=(mapped_value_x, mapped_value_y)).start()


def kirispes(center_x,center_y):
    if not processing_left:
        mapped_value_x = arduino_map(center_y, 0, height, 200, 0)
        mapped_value_y = arduino_map(center_x, 0, width , 0, 100)
        threading.Thread(target=send_data_with_feedback_left, args=(mapped_value_x, mapped_value_y)).start()


def kananspes(center_x,center_y):
    print(center_x,center_y)
    if not processing_right:
        mapped_value_x = arduino_map(center_y, 0, height, 0, 200)
        mapped_value_y = arduino_map(center_x, 280, width , 75, 0)
        threading.Thread(target=send_data_with_feedback_right, args=(mapped_value_x, mapped_value_y)).start()


def kiri(warped):
    global last_mapped_x,last_mapped_y
    height, width, _ = warped.shape
    if not processing_left:  # Only process if not currently processing another object
        x1, y1, x2, y2 = box.xyxy[0]

        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        if center_x < int(width/2):
        #     center_x_real = center_x - (int(width / 2))
            mapped_value_x = arduino_map(center_y, 0, height, 200, 0)
            mapped_value_y = arduino_map(center_x, 0, width , 0, 100)

        #     # Only send data if the change is significant
            if (last_mapped_x is None or 
                abs(mapped_value_x - last_mapped_x) > threshold or 
                abs(mapped_value_y - last_mapped_y) > threshold):

                # Update last known positions
                last_mapped_x = mapped_value_x
                last_mapped_y = mapped_value_y

                # Use threading to send data without blocking
                threading.Thread(target=send_data_with_feedback_left, args=(mapped_value_x, mapped_value_y)).start()


jumlahkiri=[]
jumlahkanan=[]
DPI = 300
width_cm = 60
height_cm = 40
width_px = 1080
height_px = 720

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)



threshold = 10  # Define a threshold for significant changes
def find_min_x_by_color(array):
    # Prioritas urutan warna
    color_priority = {"merah": 1, "kuning": 2, "hijau": 3, "biru": 4}

    # Urutkan array berdasarkan warna dan nilai x terkecil
    sorted_array = sorted(
        array, 
        key=lambda pair: (color_priority.get(pair[2].lower(), 5), pair[0])
    )
    
    # Ambil pasangan dengan nilai x terkecil dari hasil yang sudah diurutkan
    min_x_pair = sorted_array[0]  # Hanya ambil data dengan x terkecil dari urutan yang sudah diurutkan

    return min_x_pair


def find_max_x_by_color(array):
    # Prioritas urutan warna
    color_priority = {"biru": 1, "hijau": 2, "kuning": 3, "merah": 4}

    # Urutkan array berdasarkan prioritas warna dan nilai x terbesar
    sorted_array = sorted(
        array, 
        key=lambda pair: (color_priority.get(pair[2].lower(), 5), -pair[0])
    )
    
    # Ambil data dengan nilai x terbesar dari hasil yang sudah diurutkan
    max_x_pair = sorted_array[0]  # Hanya ambil data dengan x terbesar dan warna sesuai prioritas

    return max_x_pair

def boundingbox():
    last_mapped_x=None
    last_mapped_y=None
    for _, (confidence, box) in detected_objects.items():
    # if not processing_left:  # Only process if not currently processing another object
        x1, y1, x2, y2 = box.xyxy[0]
        class_index = int(box.cls[0]) 
        label = model.names[class_index] 
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        mapped_value_x = arduino_map(center_y, 0, height, 200, 0)
        mapped_value_y = arduino_map(center_x, 0, width , 0, 100)
        if (last_mapped_x is None or 
            abs(mapped_value_x - last_mapped_x) > threshold or 
            abs(mapped_value_y - last_mapped_y) > threshold):

            # Update last known positions
            last_mapped_x = mapped_value_x
            last_mapped_y = mapped_value_y

            # Use threading to send data without blocking
            # threading.Thread(target=send_data_with_feedback_left, args=(mapped_value_x, mapped_value_y)).start()
        
    # Pisahkan berdasarkan huruf besar pertama
        for i, char in enumerate(label):
            if char.isupper():
                bentuk = label[:i]  # Bagian sebelum huruf besar (bentuk)
                warna = label[i:] 
        cv2.rectangle(warped, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.circle(warped, (center_x, center_y), 5, (0, 0, 255), -1)
        text = f'{warna} ({int(mapped_value_x)}, {int(mapped_value_y)}) - {confidence:.2f}'
        cv2.putText(warped, text, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    # print("jalan")

    mask = detect_green_border(frame)
    contours = get_contours(mask)
    if contours:
        largest_contour = get_largest_contour(contours)
        approx = get_four_points_from_contour(largest_contour)
        if len(approx) == 4:
            warped = four_point_transform(frame, approx.reshape(4, 2), width_px, height_px)
            results = model(warped,verbose=False)
            height, width, _ = warped.shape

            # Filter out the bounding box with the highest confidence for each object
            detected_objects = {}
            for r in results:
                for box in r.boxes:
                    class_index = int(box.cls[0])
                    confidence = box.conf[0]

                    if class_index not in detected_objects or confidence > detected_objects[class_index][0]:
                        detected_objects[class_index] = (confidence, box)
            # print(len(detected_objects))
            boundingbox()
            # Process and send the bounding box data
            if arduino_left is not None and arduino_right is not None:

                if(len(detected_objects)==1):
                    print("satu")
                    for _, (confidence, box) in detected_objects.items():
                        kanan(warped=warped)
                        kiri(warped=warped)
                elif(len(detected_objects)>1):
                    print("oke")
                    jumlahkanan=[]
                    jumlahkiri=[]
                    semua=[]
                    for _, (confidence, box) in detected_objects.items():
                       
                        class_index = int(box.cls[0]) 
                        label = model.names[class_index]
                        for i, char in enumerate(label):
                            if char.isupper():
                                bentuk = label[:i]  # Bagian sebelum huruf besar (bentuk)
                                warna = label[i:] 
                        x1, y1, x2, y2 = box.xyxy[0]
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        if (last_mapped_x is None or 
                            abs(center_x - last_mapped_x) > threshold or 
                            abs(center_y - last_mapped_y) > threshold):

                            # Update last known positions
                            last_mapped_x = center_x
                            last_mapped_y = center_y
                            semua.append([center_x,center_y,warna])
                            if(center_x<int(width/2)):
                                jumlahkiri.append([center_x,center_y,warna])
                            else:
                                jumlahkanan.append([center_x,center_y,warna])
                    print(len(jumlahkiri),len(jumlahkanan))
                    if(len(jumlahkiri)==len(jumlahkanan)):
                        if(len(semua)!=0):
                            print(semua)
                            max_pair = find_max_x_by_color(jumlahkanan)
                            min_pair = find_min_x_by_color(jumlahkiri)
                            kananspes(center_x=max_pair[0],center_y=max_pair[1])
                            kirispes(center_x=min_pair[0],center_y=min_pair[1]) 
                    else:
                        
                        if(len(jumlahkanan)==0 and len(jumlahkiri)==1):
                            print("121")
                            kiri(warped=warped)
                        elif(len(jumlahkanan)==1 and len(jumlahkiri)==0):
                            print("122")
                            kanan(warped=warped)
                        else:
                            print("dua")
                            print(semua)
                            # if(len(jumlahkanan)!=0):
                            max_pair = find_max_x_pair(semua)
                            min_pair = find_min_x_pair(semua)
                            kananspes(center_x=max_pair[0],center_y=max_pair[1])
                            # if(len(jumlahkiri)!=0):
                                
                            kirispes(center_x=min_pair[0],center_y=min_pair[1])
                        # elif(len(jumlahkanan)>=1 and len(jumlahkiri)>=1):
                        #     print("tiga")
                        #     kanan(warped=warped)
                        #     kiri(warped=warped)
            elif arduino_left is None and arduino_right is not None  :
                print("kiriada")
                if(len(detected_objects)>=1):
                    semua=[]
                    for _, (confidence, box) in detected_objects.items():
                        class_index = int(box.cls[0]) 
                        label = model.names[class_index]
                        for i, char in enumerate(label):
                            if char.isupper():
                                bentuk = label[:i]  # Bagian sebelum huruf besar (bentuk)
                                warna = label[i:] 
                        x1, y1, x2, y2 = box.xyxy[0]
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        if (last_mapped_x is None or 
                            abs(center_x - last_mapped_x) > threshold or 
                            abs(center_y - last_mapped_y) > threshold):

                            # Update last known positions
                            last_mapped_x = center_x
                            last_mapped_y = center_y
                            semua.append([center_x,center_y,warna])
                    if(len(semua)==1):
                        kananspes(center_x=center_x,center_y=center_y)
                    elif(len(semua)>1):
                        max_pair = find_max_x_by_color(semua)
                        min_pair = find_min_x_by_color(semua)
                        kananspes(center_x=max_pair[0],center_y=max_pair[1])
            elif arduino_left is not None and arduino_right is None:
                print("kananada")
                if(len(detected_objects)>=1):
                    semua=[]
                    for _, (confidence, box) in detected_objects.items():
                        class_index = int(box.cls[0]) 
                        label = model.names[class_index]
                        for i, char in enumerate(label):
                            if char.isupper():
                                bentuk = label[:i]  # Bagian sebelum huruf besar (bentuk)
                                warna = label[i:] 
                        x1, y1, x2, y2 = box.xyxy[0]
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        if (last_mapped_x is None or 
                            abs(center_x - last_mapped_x) > threshold or 
                            abs(center_y - last_mapped_y) > threshold):

                            # Update last known positions
                            last_mapped_x = center_x
                            last_mapped_y = center_y
                            semua.append([center_x,center_y,warna])
                    if(len(semua)==1):
                        kirispes(center_x=center_x,center_y=center_y)
                    elif(len(semua)>1):
                        max_pair = find_max_x_by_color(semua)
                        min_pair = find_min_x_by_color(semua)
                        kirispes(center_x=max_pair[0],center_y=max_pair[1])

                        
                         


                    
                    
            cv2.imshow("Warped", warped)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
