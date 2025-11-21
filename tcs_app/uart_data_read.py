import serial
import re

ser = serial.Serial(port='COM6', baudrate=115200) #change to appropriate port per device

#data arrays
#init data for dpg graphs
nsamples = 10

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

time_x=[0.0]*nsamples
data_arrays = []

#LIST OF DATA
em_curr = [0.0]*nsamples
em_volt = [0.0]*nsamples
motor_speed = [0.0]*nsamples
bus_curr = [0.0]*nsamples
bus_volt = [0.0]*nsamples
inv_en_state = [0.0]*nsamples
inv_state = [0.0]*nsamples
torq_feed = [0.0]*nsamples
cmded_torque = [0.0]*nsamples
torq_cmd = [0.0]*nsamples
speed_cmd = [0.0]*nsamples
speed_mode_en = [0.0]*nsamples
throttle_perc = [0.0]*nsamples
brake_perc = [0.0]*nsamples
vcu_fl = [0.0]*nsamples
vcu_fr = [0.0]*nsamples
vcu_rl = [0.0]*nsamples
vcu_rr = [0.0]*nsamples
term_sense_lost = [0.0]*nsamples
batt_low_volt = [0.0]*nsamples
regen_mode = [0.0]*nsamples
regen_max_torq = [0.0]*nsamples
speed_kph = [0.0]*nsamples
steer_angle = [0.0]*nsamples
high_cell_temp = [0.0]*nsamples
high_cell_volt = [0.0]*nsamples
low_cell_volt = [0.0]*nsamples
high_cell_temp_dc = [0.0]*nsamples
PL_total_err = [0.0]*nsamples
PL_prop = [0.0]*nsamples
PL_integral = [0.0]*nsamples
PL_torq_cmd = [0.0]*nsamples
LC_torq_cmd = [0.0]*nsamples
LC_slip = [0.0]*nsamples
LC_pid = [0.0]*nsamples
LC_prop = [0.0]*nsamples
LC_integral = [0.0]*nsamples
LC_total_err = [0.0]*nsamples
BMS_state = [0.0]*nsamples
BMS_main_pos = [0.0]*nsamples
BMS_main_neg = [0.0]*nsamples
pack_volt = [0.0]*nsamples
BMS_charge = [0.0]*nsamples
BMS_high_temp = [0.0]*nsamples
humidity = [0.0]*nsamples
temp = [0.0]*nsamples

# data_arrays.append(em_curr) #0
# data_arrays.append(em_volt) #1
# data_arrays.append(motor_speed) #2
# data_arrays.append(bus_curr) #3
# data_arrays.append(bus_volt) #4
# data_arrays.append(inv_en_state) #5
# data_arrays.append(inv_state) #6
# data_arrays.append(torq_feed) #7
# data_arrays.append(cmded_torque) #8
# data_arrays.append(torq_cmd) #9
# data_arrays.append(speed_cmd) #10
# data_arrays.append(speed_mode_en) #11
# data_arrays.append(throttle_perc) #12
# data_arrays.append(brake_perc) #13
# data_arrays.append(vcu_fl) #14
# data_arrays.append(vcu_fr) #15
# data_arrays.append(vcu_rl) #16
# data_arrays.append(vcu_rr) #17
# data_arrays.append(term_sense_lost) #18
# data_arrays.append(batt_low_volt) #19
# data_arrays.append(regen_mode) #20
# data_arrays.append(regen_max_torq) #21
# data_arrays.append(speed_kph) #22
# data_arrays.append(steer_angle) #23
# data_arrays.append(high_cell_temp) #24
# data_arrays.append(high_cell_volt) #25
# data_arrays.append(low_cell_volt) #26
# data_arrays.append(high_cell_temp_dc) #27
# data_arrays.append(PL_total_err) #28
# data_arrays.append(PL_prop) #29
# data_arrays.append(PL_integral) #30
# data_arrays.append(PL_torq_cmd) #31
# data_arrays.append(LC_torq_cmd) #32
# data_arrays.append(LC_slip) #33
# data_arrays.append(LC_pid) #34
# data_arrays.append(LC_prop) #35
# data_arrays.append(LC_integral) #36
# data_arrays.append(LC_total_err) #37
# data_arrays.append(BMS_state) #38
# data_arrays.append(BMS_main_pos) #39
# data_arrays.append(BMS_main_neg) #40
# data_arrays.append(pack_volt) #41
# data_arrays.append(BMS_charge) #42
# data_arrays.append(BMS_high_temp) #43
# data_arrays.append(humidity) #44
# data_arrays.append(temp) #45


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
        #print(res)
        #print(data_arrays)

        #fault check
        if (res[DATA_TYPE] in fault_str):
            fault_index = fault_str.index(res[DATA_TYPE])
            faults[fault_index] = 1

        # if (res[DATA_TYPE] in data_str):
        #     data_index = data_str.index(res[DATA_TYPE])
        #     data_arrays[data_index].append(int(res[DATA_VALUE]))
        #     count = count + 1
            #print(data_arrays[data_index][-1])
            #print(pack_volt[-1])
        
        match res[DATA_TYPE]: #add more cases for future data       
            #MCM
            case 'EMeter_Current':
                em_curr.append(int(res[DATA_VALUE]))
                count = count + 1
                print(res)
                print('em_curr: ', em_curr[-1])

            case 'EMeter_Voltage':
                em_volt.append(int(res[DATA_VALUE]))
                count = count + 1
                print(res)
                print('em_volt: ', em_volt[-1])

            case 'MCM_Motor_Speed':
                motor_speed.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_DCBus_Current':
                bus_curr.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_DCBus_Voltage':
                bus_volt.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_IntInvert_EnableState':
                inv_en_state.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_IntInverter_State':
                inv_state.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_Torque_Feedback':
                torq_feed.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_Commanded_Torque':
                cmded_torque.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_Torque_Command':
                torq_cmd.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_Speed_Command':
                speed_cmd.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_Speed_Mode_Enable':
                speed_mode_en.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'TPS1_Throttle_Percent':
                throttle_perc.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'BPS0_Brake_Percent':
                brake_perc.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_WSS_FL_S':
                vcu_fl.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_WSS_FR_S':
                vcu_fr.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_WSS_RL_S':
                vcu_rl.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_WSS_RR_S':
                vcu_rr.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_NOTICE_HVIL_TermSenseLost':
                term_sense_lost.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'Battery_Low_Voltage':
                batt_low_volt.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_MCM_RegenMode':
                regen_mode.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'MCM_Speed_VCU_MCM_Regen_MaxTorqueNmCommand':
                regen_max_torq.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'Speed_KPH':
                speed_kph.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'Steering_Angle':
                steer_angle.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_BMS_HighestCellTemp':
                high_cell_temp.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_BMS_HighestCellVoltage':
                high_cell_volt.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_BMS_LowestCellVoltage':
                low_cell_volt.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_BMS_HighestCellTemp_dC':
                high_cell_temp_dc.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_POWERLIMIT_PID_getTotalError':
                PL_total_err.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_POWERLIMIT_PID_getProportional':
                PL_prop.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_POWERLIMIT_PID_getIntegral':
                PL_integral.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_POWERLIMIT_getTorqueCommand_Nm':
                PL_torq_cmd.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_LaunchControl_getTorqueCommand_Nm':
                LC_torq_cmd.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_LaunchControl_getSlipRatioScaled':
                LC_slip.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_LaunchControl_getPidOutput':
                LC_pid.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_LaunchControl_PID_Proportional':
                LC_prop.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_LaunchControl_PID_Integral':
                LC_integral.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'VCU_LaunchControl_PID_TotalError':
                LC_total_err.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'BMS_Current_State':
                BMS_state.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'BMS_Main_Contactor_Positive_Closed':
                BMS_main_pos.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'BMS_Main_Contactor_Negative_Closed':
                BMS_main_neg.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'Pack_Voltage':
                pack_volt.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'BMS_State_Of_Charge':
                BMS_charge.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'BMS_Highest_Cell_Temperature':
                BMS_high_temp.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'Humidity':
                humidity.append(int(res[DATA_VALUE]))
                count = count + 1

            case 'Temperature':
                temp.append(int(res[DATA_VALUE]))
                count = count + 1
        
    
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
