import serial
import re

ser = serial.Serial(port='COM6', baudrate=115200) #change to appropriate port per device

samples = 100
#data arrays
em_current = [0.0]*samples
em_volt = [0.0]*samples
motor_speed = [0.0]*samples
bus_current = [0.0]*samples
torque_feed = [0.0]*samples
command_torque = [0.0]*samples
throttle_perc = [0.0]*samples
steer_angle = [0.0]*samples
pack_volt = [0.0]*samples
volt_info = [0.0]*samples
ground = [0.0]*samples

#not graphed/for digital display: All VCU data/fault, BMS_fault

global count 
count = 0 #for testing latency


def update_data():
    try:
        value = ser.readline()
        valueInString = value.decode('UTF-8', errors='ignore')
        res = re.split(r'[: ]', valueInString)
        #print(res)
        #count = count + 1

        match res[0]: #add more cases for future data
            case 'EMeter_Current':
                em_current.append(int(res[2]))
                #print('em_current:', em_current)

            case 'EMeter_Voltage':
                em_volt.append(int(res[2]))
                #print('em_volt:', em_volt)

            case 'MCM_Motor_Speed':
                motor_speed.append(int(res[2]))
                #print('motor_speed:', motor_speed)

            case 'MCM_DC_Bus_Current':
                bus_current.append(int(res[2]))
                #print('bus_current:', bus_current)

            case 'MCM_Torque_Feedback':
                torque_feed.append(int(res[2]))
                #print('torque_feed:', torque_feed)

            case 'MCM_Commanded_Torque':
                command_torque.append(int(res[2]))
                #print('command_torque:', command_torque)

            case 'TPS0ThrottlePercent0FF':
                throttle_perc.append(int(res[2]))
                #print('throttle_perc:', throttle_perc)

            case 'Steering_Angle':
                steer_angle.append(int(res[2]))
                #print('steer_angle:', steer_angle)

            case 'Pack_Voltage':
                pack_volt.append(int(res[2]))
                #print('pack_volt:', pack_volt)

            case 'MCM_Voltage_Info':
                volt_info.append(int(res[2]))
                #print('volt_info:', volt_info)

            case 'Ground_Speed':
                ground.append(int(res[2]))
                #print('ground: ', ground)
    
    except UnicodeDecodeError as e:
        print(f"[Decode Error] Invalid start byte encountered: {e}")
    except IndexError as e:
        print(f"[Index Error] Incomplete data received: {e}")
    except ValueError as e:
        print(f"[Value Error] Could not convert to integer: {e}")
    except Exception as e:
        print(f"[Unexpected Error] {e}")

    #print(type(ground[0]))
    #print(res[4])
    #count = count + 1
    #print(count)


def send_data(prompt):
    p_byte = prompt.encode('utf-8')
    ser.write(p_byte)
    print(p_byte)

'''
while True:
    value = ser.readline()
    valueInString = str(value, 'UTF-8')
    res = re.split(r'[: ]', valueInString)
    print(res)
'''