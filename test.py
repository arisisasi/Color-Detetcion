import tkinter as tk
from tkinter import ttk
import serial
import time
import os

# Mengatur koneksi serial ke Arduino
ser = serial.Serial('COM10', 9600)  # Sesuaikan COM port dengan port Anda
time.sleep(2)  # Memberi waktu untuk inisialisasi koneksi serial

# Definisi titik dan label
coordinates = [
    (0, 0), (25, 0), (50, 0), (75, 0), (100, 0), (125, 0), (150, 0), (175, 0), (200, 0),
    (0, 12.5), (25, 12.5), (50, 12.5), (75, 12.5), (100, 12.5), (125, 12.5), (150, 12.5), (175, 12.5), (200, 12.5),
    (0, 25), (25, 25), (50, 25), (75, 25), (100, 25), (125, 25), (150, 25), (175, 25), (200, 25),
    (0, 37.5), (25, 37.5), (50, 37.5), (75, 37.5), (100, 37.5), (125, 37.5), (150, 37.5), (175, 37.5), (200, 37.5),
    (0, 50), (25, 50), (50, 50), (75, 50), (100, 50), (125, 50), (150, 50), (175, 50), (200, 50)
]
labels = [chr(65 + i) if i < 26 else chr(65 + (i // 26 - 1)) + chr(65 + (i % 26)) for i in range(45)]
coord_dict = {label: coord for coord, label in zip(coordinates, labels)}

# Nama file untuk menyimpan data
data_file = "servo6dof_data.txt"

def read_last_index():
    if os.path.exists(data_file):
        with open(data_file, "r") as file:
            lines = file.readlines()
            if lines:
                last_line = lines[-1]
                last_label = last_line.split("Position: ")[-1].split(" ")[0]
                return labels.index(last_label) + 1
    return 0

# Inisialisasi indeks posisi alfabet
current_index = read_last_index()

def send_command():
    # Mengirimkan nilai posisi servo ke Arduino
    base = slider_base.get()
    shoulder = slider_shoulder.get()
    elbow = slider_elbow.get()
    grip = slider_grip.get()
    wrist = slider_wrist.get()

    command = f'{base},{shoulder},{elbow},{grip},{wrist}\n'
    ser.write(command.encode('utf-8'))

    # Memperbarui label dengan nilai sudut
    label_output.config(text=f"Base: {base}°\nShoulder: {shoulder}°\nElbow: {elbow}°\nGrip: {grip}°\nWrist: {wrist}°")

def save_data():
    global current_index
    if current_index >= len(labels):
        return  # Semua data telah disimpan

    # Mendapatkan nilai posisi servo
    base = slider_base.get()
    shoulder = slider_shoulder.get()
    elbow = slider_elbow.get()
    grip = slider_grip.get()
    wrist = slider_wrist.get()

    # Mendapatkan posisi alfabet dan koordinat
    position_label = labels[current_index]
    position_coord = coord_dict[position_label]

    # Menyimpan data ke file teks
    with open(data_file, "a") as file:
        file.write(f"Base: {int(base)}, Shoulder: {int(shoulder)}, Elbow: {int(elbow)}, Grip: {int(grip)}, Wrist: {int(wrist)}, Position: {position_label} ({position_coord})\n")

    # Meningkatkan indeks untuk posisi alfabet berikutnya
    current_index += 1

# Fungsi untuk menambah atau mengurangi nilai slider
def increase_value(slider):
    current_value = slider.get()
    if current_value < 180:
        slider.set(current_value + 1)

def decrease_value(slider):
    current_value = slider.get()
    if current_value > 0:
        slider.set(current_value - 1)

# Membuat window utama
root = tk.Tk()
root.title("Kontrol Robot Arm")

# Menambahkan slider dan tombol untuk masing-masing servo
def create_servo_control(label_text, pin, slider_var):
    frame = ttk.Frame(root)
    frame.pack(pady=5)
    
    label = ttk.Label(frame, text=f"{label_text} (Pin {pin})")
    label.grid(row=0, column=0)

    button_dec = ttk.Button(frame, text="-", command=lambda: [decrease_value(slider_var), send_command()])
    button_dec.grid(row=0, column=1)

    slider = ttk.Scale(frame, from_=0, to=180, orient='horizontal', length=200, variable=slider_var, command=lambda x: send_command())
    slider.grid(row=0, column=2)

    button_inc = ttk.Button(frame, text="+", command=lambda: [increase_value(slider_var), send_command()])
    button_inc.grid(row=0, column=3)

    return slider

# Inisialisasi variabel slider
slider_base = tk.DoubleVar(value=73)
slider_shoulder = tk.DoubleVar(value=93)
slider_elbow = tk.DoubleVar(value=119)
slider_grip = tk.DoubleVar(value=57)
slider_wrist = tk.DoubleVar(value=90)

# Membuat kontrol servo
create_servo_control("Base", 3, slider_base)
create_servo_control("Shoulder", 5, slider_shoulder)
create_servo_control("Elbow", 6, slider_elbow)
create_servo_control("Grip", 9, slider_grip)
create_servo_control("Wrist", 10, slider_wrist)

# Menambahkan label untuk menampilkan nilai sudut
label_output = ttk.Label(root, text=f"Base: {slider_base.get()}°\nShoulder: {slider_shoulder.get()}°\nElbow: {slider_elbow.get()}°\nGrip: {slider_grip.get()}°\nWrist: {slider_wrist.get()}°")
label_output.pack()

# Menambahkan tombol untuk menyimpan data
button_save = ttk.Button(root, text="Simpan Data", command=save_data)
button_save.pack()

# Menjalankan loop utama GUI
root.mainloop()

# Menutup koneksi serial saat program berakhir
ser.close()
