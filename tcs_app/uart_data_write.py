import serial

ser = serial.Serial(port='COM7', baudrate=115200) #change to appropriate port per device

def send_data(prompt):
    if prompt == 1:
        print("prompt 1")