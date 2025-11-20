import serial
import re

ser = serial.Serial(port='COM6', baudrate=115200) #change to appropriate port per device

#samples = 100

#data arrays
#init data for dpg graphs
nsamples = 5
DATA_INIT = [0.0]*nsamples

data_str = ['EMeter_Current', 'EMeter_Voltage', 'MCM_Motor_Speed', 'MCM_DCBus_Current', 
            'MCM_DCBus_Voltage', 'MCM_IntInvert_EnableState', 'MCM_IntInverter_State',
            'MCM_Torque_Feedback', 'MCM_Commanded_Torque', 'MCM_Torque_Command', 
            'MCM_Speed_Command', 'MCM_Speed_Mode_Enable', 'TPS1_Throttle_Percent',
            'BPS0_Brake_Percent', 'VCU_WSS_FL_S', 'VCU_WSS_FR_S', 'VCU_WSS_RL_S',
            'VCU_WSS_RR_S', 'VCU_NOTICE_HVIL_TermSenseLost', 'Battery_Low_Voltage',
            'VCU_MCM_RegenMode', 'VCU_MCM_Regen_MaxTorqueNm', 'Speed_KPH', 'Steering_Angle',
            'VCU_BMS_HighestCellTemp', 'VCU_BMS_HighestCellVoltage', 'VCU_BMS_LowestCellVoltage',
            'VCU_BMS_HighestCellTemp_dC', 'VCU_POWERLIMIT_PID_getTotalError', 'VCU_POWERLIMIT_PID_getProportional',
            'VCU_POWERLIMIT_PID_getIntegral', 'VCU_POWERLIMIT_getTorqueCommand_Nm',
            'VCU_LaunchControl_getTorqueCommand_Nm', 'VCU_LaunchControl_getSlipRatioScaled',
            'VCU_LaunchControl_getPidOutput', 'VCU_LaunchControl_PID_Proportional', 
            'VCU_LaunchControl_PID_Integral', 'VCU_LaunchControl_PID_TotalError',
            'BMS_Current_State', 'BMS_Main_Contactor_Positive_Closed', 'BMS_Main_Contactor_Negative_Closed',
            'Pack_Voltage', 'BMS_State_Of_Charge', 'BMS_Highest_Cell_Temperature',
            'Humidity', 'Temperature']

time_x=DATA_INIT
data_arrays = []

#LIST OF DATA
em_curr = DATA_INIT
em_volt = DATA_INIT
motor_speed = DATA_INIT
bus_curr = DATA_INIT
bus_volt = DATA_INIT
inv_en_state = DATA_INIT
inv_state = DATA_INIT
torq_feed = DATA_INIT
cmded_torque = DATA_INIT
torq_cmd = DATA_INIT
speed_cmd = DATA_INIT
speed_mode_en = DATA_INIT
throttle_perc = DATA_INIT
brake_perc = DATA_INIT
vcu_fl = DATA_INIT
vcu_fr = DATA_INIT
vcu_rl = DATA_INIT
vcu_rr = DATA_INIT
term_sense_lost = DATA_INIT
batt_low_volt = DATA_INIT
regen_mode = DATA_INIT
regen_max_torq = DATA_INIT
speed_kph = DATA_INIT
steer_angle = DATA_INIT
high_cell_temp = DATA_INIT
high_cell_volt = DATA_INIT
low_cell_volt = DATA_INIT
high_cell_temp_dc = DATA_INIT
PL_total_err = DATA_INIT
PL_prop = DATA_INIT
PL_integral = DATA_INIT
PL_torq_cmd = DATA_INIT
LC_torq_cmd = DATA_INIT
LC_slip = DATA_INIT
LC_pid = DATA_INIT
LC_prop = DATA_INIT
LC_integral = DATA_INIT
LC_total_err = DATA_INIT
BMS_state = DATA_INIT
BMS_main_pos = DATA_INIT
BMS_main_neg = DATA_INIT
pack_volt = DATA_INIT
BMS_charge = DATA_INIT
BMS_high_temp = DATA_INIT
humidity = DATA_INIT
temp = DATA_INIT

data_arrays.append(em_curr) #0
data_arrays.append(em_volt) #1
data_arrays.append(motor_speed) #2
data_arrays.append(bus_curr) #3
data_arrays.append(bus_volt) #4
data_arrays.append(inv_en_state) #5
data_arrays.append(inv_state) #6
data_arrays.append(torq_feed) #7
data_arrays.append(cmded_torque) #8
data_arrays.append(torq_cmd) #9
data_arrays.append(speed_cmd) #10
data_arrays.append(speed_mode_en) #11
data_arrays.append(throttle_perc) #12
data_arrays.append(brake_perc) #13
data_arrays.append(vcu_fl) #14
data_arrays.append(vcu_fr) #15
data_arrays.append(vcu_rl) #16
data_arrays.append(vcu_rr) #17
data_arrays.append(term_sense_lost) #18
data_arrays.append(batt_low_volt) #19
data_arrays.append(regen_mode) #20
data_arrays.append(regen_max_torq) #21
data_arrays.append(speed_kph) #22
data_arrays.append(steer_angle) #23
data_arrays.append(high_cell_temp) #24
data_arrays.append(high_cell_volt) #25
data_arrays.append(low_cell_volt) #26
data_arrays.append(high_cell_temp_dc) #27
data_arrays.append(PL_total_err) #28
data_arrays.append(PL_prop) #29
data_arrays.append(PL_integral) #30
data_arrays.append(PL_torq_cmd) #31
data_arrays.append(LC_torq_cmd) #32
data_arrays.append(LC_slip) #33
data_arrays.append(LC_pid) #34
data_arrays.append(LC_prop) #35
data_arrays.append(LC_integral) #36
data_arrays.append(LC_total_err) #37
data_arrays.append(BMS_state) #38
data_arrays.append(BMS_main_pos) #39
data_arrays.append(BMS_main_neg) #40
data_arrays.append(pack_volt) #41
data_arrays.append(BMS_charge) #42
data_arrays.append(BMS_high_temp) #43
data_arrays.append(humidity) #44
data_arrays.append(temp) #45

# em_current = [0.0]*samples
# em_volt = [0.0]*samples
# motor_speed = [0.0]*samples
# bus_current = [0.0]*samples
# torque_feed = [0.0]*samples
# command_torque = [0.0]*samples
# throttle_perc = [0.0]*samples
# steer_angle = [0.0]*samples
# pack_volt = [0.0]*samples
# volt_info = [0.0]*samples
# ground = [0.0]*samples

#fault array
fault_str = ['VCU_Fault_TPS_OutOfRange', 'VCU_Fault_BPS_OutOfRange', 'VCU_FAULT_TPS_OutOfSync', 
             'VCU_FAULT_TPSBPS_Implausible', 'VCU_FAULT_BSPD_SoftFault', 'VCU_FAULT_LVS_BatteryLow',
             'BMS_Imminent_Contactor_Opening_Warning', 'BMS_Cell_Under_Voltage_Fault', 
             'BMS_Cell_Over_Temperature_Fault', 'BMS_Pack_Under_Voltage_Fault',
             'BMS_Isolation_Leakage_Fault', 'BMS_Precharge_Fault', 'BMS_Failed_Thermistor_Fault']

faults = [0]*(len(fault_str))

count = 0 #for testing latency

#indices
DATA_VALUE = 4
DATA_TYPE = 2

def update_data():
    global count 
    try:
        value = ser.readline()
        valueInString = value.decode('UTF-8', errors='ignore')
        res = re.split(r'[: ]', valueInString)
        print(res)
        #print(data_arrays)

        #fault check
        if (res[DATA_TYPE] in fault_str):
            fault_index = fault_str.index(res[DATA_TYPE])
            faults[fault_index] = 1

        if (res[DATA_TYPE] in data_str):
            data_index = data_str.index(res[DATA_TYPE])
            data_arrays[data_index].append(int(res[DATA_VALUE]))
            count = count + 1
        
        # match res[DATA_TYPE]: #add more cases for future data       
        #     #MCM
        #     case 'EMeter_Current':
        #         data_arrays[0].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'EMeter_Voltage':
        #         data_arrays[1].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_Motor_Speed':
        #         data_arrays[2].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_DCBus_Current':
        #         data_arrays[3].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_DCBus_Voltage':
        #         data_arrays[4].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_IntInvert_EnableState':
        #         data_arrays[5].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_IntInverter_State':
        #         data_arrays[6].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_Torque_Feedback':
        #         data_arrays[7].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_Commanded_Torque':
        #         data_arrays[8].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_Torque_Command':
        #         data_arrays[9].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_Speed_Command':
        #         data_arrays[10].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_Speed_Mode_Enable':
        #         data_arrays[11].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'TPS1_Throttle_Percent':
        #         data_arrays[12].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'BPS0_Brake_Percent':
        #         data_arrays[13].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_WSS_FL_S':
        #         data_arrays[14].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_WSS_FR_S':
        #         data_arrays[15].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_WSS_RL_S':
        #         data_arrays[16].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_WSS_RR_S':
        #         data_arrays[17].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_NOTICE_HVIL_TermSenseLost':
        #         data_arrays[18].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'Battery_Low_Voltage':
        #         data_arrays[19].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_MCM_RegenMode':
        #         data_arrays[20].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'MCM_Speed_VCU_MCM_Regen_MaxTorqueNmCommand':
        #         data_arrays[21].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'Speed_KPH':
        #         data_arrays[22].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'Steering_Angle':
        #         data_arrays[23].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_BMS_HighestCellTemp':
        #         data_arrays[24].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_BMS_HighestCellVoltage':
        #         data_arrays[25].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_BMS_LowestCellVoltage':
        #         data_arrays[26].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_BMS_HighestCellTemp_dC':
        #         data_arrays[27].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_POWERLIMIT_PID_getTotalError':
        #         data_arrays[28].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_POWERLIMIT_PID_getProportional':
        #         data_arrays[29].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_POWERLIMIT_PID_getIntegral':
        #         data_arrays[30].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_POWERLIMIT_getTorqueCommand_Nm':
        #         data_arrays[31].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_LaunchControl_getTorqueCommand_Nm':
        #         data_arrays[32].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_LaunchControl_getSlipRatioScaled':
        #         data_arrays[33].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_LaunchControl_getPidOutput':
        #         data_arrays[34].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_LaunchControl_PID_Proportional':
        #         data_arrays[35].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_LaunchControl_PID_Integral':
        #         data_arrays[36].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'VCU_LaunchControl_PID_TotalError':
        #         data_arrays[37].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'BMS_Current_State':
        #         data_arrays[38].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'BMS_Main_Contactor_Positive_Closed':
        #         data_arrays[39].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'BMS_Main_Contactor_Negative_Closed':
        #         data_arrays[40].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'Pack_Voltage':
        #         data_arrays[41].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'BMS_State_Of_Charge':
        #         data_arrays[42].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'BMS_Highest_Cell_Temperature':
        #         data_arrays[43].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'Humidity':
        #         data_arrays[44].append(int(res[DATA_VALUE]))
        #         count = count + 1

        #     case 'Temperature':
        #         data_arrays[45].append(int(res[DATA_VALUE]))
        #         count = count + 1
        
    
    except UnicodeDecodeError as e:
        print(f"[Decode Error] Invalid start byte encountered: {e}")
    except IndexError as e:
        print(f"[Index Error] Incomplete data received: {e}")
    except ValueError as e:
        print(f"[Value Error] Could not convert to integer: {e}")
    except Exception as e:
        print(f"[Unexpected Error] {e}")


def send_data(prompt):
    p_byte = prompt.encode('utf-8')
    ser.write(p_byte)
    #print(p_byte) #for testing
