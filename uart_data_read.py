import serial
import re

ser = serial.Serial(port='COM7', baudrate=115200) #change to appropriate port per device
ground = []
volt_in = []
count = 0

while True:
    value = ser.readline()
    valueInString = str(value, 'UTF-8')
    res = re.split(r'[: ]', valueInString)
    match res[2]: #add more cases for future data
        case 'Ground_Speed':
            ground.append(int(res[4]))
            print('ground: ', ground)
        case 'MCM_Voltage_Info':
            volt_in.append(int(res[4]))
            print('volt_in:', volt_in)

    #print(type(ground[0]))
    #print(res[4])
    #count = count + 1
    #print(count)