import serial
import re

ser = serial.Serial(port='COM7', baudrate=115200) #change to appropriate port per device

#data arrays
em_current = []
em_volt = []
motor_speed = []
bus_current = []
torque_feed = []
command_torque = []
throttle_perc = []
steer_angle = []
pack_volt = []
volt_info = []
ground = []

#not graphed/for digital display: All VCU data/fault, BMS_fault

global count 
count = 0 #for testing latency


def update_data():
    value = ser.readline()
    valueInString = str(value, 'UTF-8')
    res = re.split(r'[: ]', valueInString)
    #print(res)
    #count = count + 1

    match res[0]: #add more cases for future data
        case 'EMeter_Current':
            em_current.append(int(res[2]))
            print('em_current:', em_current)

        case 'EMeter_Voltage':
            em_volt.append(int(res[2]))
            print('em_volt:', em_volt)

        case 'MCM_Motor_Speed':
            motor_speed.append(int(res[2]))
            print('motor_speed:', motor_speed)

        case 'MCM_DC_Bus_Current':
            bus_current.append(int(res[2]))
            print('bus_current:', bus_current)

        case 'MCM_Torque_Feedback':
            torque_feed.append(int(res[2]))
            print('torque_feed:', torque_feed)

        case 'MCM_Commanded_Torque':
            command_torque.append(int(res[2]))
            print('command_torque:', command_torque)

        case 'TPS0ThrottlePercent0FF':
            throttle_perc.append(int(res[2]))
            print('throttle_perc:', throttle_perc)

        case 'Steering_Angle':
            steer_angle.append(int(res[2]))
            print('steer_angle:', steer_angle)

        case 'Pack_Voltage':
            pack_volt.append(int(res[2]))
            print('pack_volt:', pack_volt)

        case 'MCM_Voltage_Info':
            volt_info.append(int(res[2]))
            print('volt_info:', volt_info)

        case 'Ground_Speed':
            ground.append(int(res[2]))
            print('ground: ', ground)

    #print(type(ground[0]))
    #print(res[4])
    #count = count + 1
    #print(count)



'''
while True:
    value = ser.readline()
    valueInString = str(value, 'UTF-8')
    res = re.split(r'[: ]', valueInString)
    print(res)
'''