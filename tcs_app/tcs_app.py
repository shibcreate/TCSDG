import dearpygui.dearpygui as dpg
import uart_data_read as udr

import time
import threading

#INIT FOR DATA GRAPHS
nsamples = 5
DATA_INIT = [0.0]*nsamples

#FULL DATA NAMES FOR TAGS
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

DATA_LEN = len(data_str)
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


#for warning/error functions, easy to change if needed
thresholds = [300, 200, 250, 0]

'''
indices for thresholds:
0 - pack volt warning
1 - pack volt danger
2 - command torque danger
3 - throttle_perc danger
'''

#fault array
fault_str = ['VCU_Fault_TPS_OutOfRange', 'VCU_Fault_BPS_OutOfRange', 'VCU_FAULT_TPS_OutOfSync', 
             'VCU_FAULT_TPSBPS_Implausible', 'VCU_FAULT_BSPD_SoftFault', 'VCU_FAULT_LVS_BatteryLow',
             'BMS_Imminent_Contactor_Opening_Warning', 'BMS_Cell_Under_Voltage_Fault', 
             'BMS_Cell_Over_Temperature_Fault', 'BMS_Pack_Under_Voltage_Fault',
             'BMS_Isolation_Leakage_Fault', 'BMS_Precharge_Fault', 'BMS_Failed_Thermistor_Fault']
FAULT_LEN = len(fault_str)
faults = [0]*(FAULT_LEN)

'''
indices for faults:
0 - VCU_Fault_TPS_OutOfRange
1 - VCU_Fault_BPS_OutOfRange
2 - VCU_FAULT_TPS_OutOfSync
3 - VCU_FAULT_TPSBPS_Implausible
4 - VCU_FAULT_BSPD_SoftFault
5 - VCU_FAULT_LVS_BatteryLow
6 - BMS_Imminent_Contactor_Opening_Warning
7 - BMS_Cell_Under_Voltage_Fault
8 - BMS_Cell_Over_Temperature_Fault
9 - BMS_Pack_Under_Voltage_Fault
10 - BMS_Isolation_Leakage_Fault
11 - BMS_Precharge_Fault
12 - BMS_Failed_Thermistor_Fault
'''

#GUI constants
GRAPH_WIDTH = 470
GRAPH_HEIGHT = 230
GRAPH_POS_XL = 0
GRAPH_POS_XR = GRAPH_POS_XL + GRAPH_WIDTH
GRAPH_POS_Y = 50
WARN_COLOR = (255, 255, 0) #yellow
DANGER_COLOR = (255, 165, 0) #orange
FAULT_COLOR = (255, 0, 0) #red

#MAIN THREAD
def update_all():
    t_u = time.time()
    while True:
        for i in range(DATA_LEN):
            udr.update_data()
        
        time_x.append(time.time() - t_u) #update time x axis
        for i in range(DATA_LEN):
            data_arrays[i].append(udr.data_arrays[i][-1])
            dpg.set_value(data_str[i], [list(time_x), list(data_arrays[i])]) #set series x and y to last nsamples

        #threshold checks
        if (data_arrays[41][-1] < thresholds[0] and data_arrays[41][-1] > thresholds[1]):
            thresh_check("pack voltage", data_arrays[41][-1], time_x[-1], 0)
        if (data_arrays[41][-1] < thresholds[1]):
            thresh_check("pack voltage", data_arrays[41][-1], time_x[-1], 1)
        if (data_arrays[8][-1] >= thresholds[2]):
            thresh_check("commanded torque", data_arrays[8][-1], time_x[-1], 1)      
        if (data_arrays[12][-1] <= thresholds[3]):
            thresh_check("throttle percent", data_arrays[12][-1], time_x[-1], 1)

        #error check
        faults = udr.faults
        for i in range(FAULT_LEN):
            if(faults[i] == 1):
                msg = f"FAULT OCCURRED OF TYPE: {fault_str[i]}"
                dpg.add_text(msg, parent='log_container', color=FAULT_COLOR) #red tuple
                dpg.set_y_scroll('log_container', 9999) #auto scroll function
                faults[i] = 0
                udr.faults[i] = 0

        #power calculations: dc_bus_current * dc_voltage
        power_calc(data_arrays[3][-1], data_arrays[4][-1])

        #fit axes
        for i in range(DATA_LEN):
            dpg.fit_axis_data(f'x_axis{i+1}')
            dpg.fit_axis_data(f'y_axis{i+1}')

dpg.create_context()


#GRAPHS WINDOWS
def create_plot(plot_label, pos_coord, xtag, ylabel, ytag, ylist, line_tag):
    with dpg.plot(label=plot_label, pos=pos_coord, height=GRAPH_HEIGHT, width=GRAPH_WIDTH):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='Time (s)', tag=xtag)
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label=ylabel, tag=ytag)
        dpg.add_line_series(x=list(time_x), y=list(ylist), parent=ytag, tag=line_tag)


with dpg.window(label='GRAPHS', tag='win', width=1000, height = 800, no_scroll_with_mouse=False):
    with dpg.tab_bar(label='tab_bar', reorderable=True):
        with dpg.tab(label='MCM', tracked=True):
            create_plot(data_str[0], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis1', 'current', 'y_axis1', data_arrays[0], data_str[0])
            create_plot(data_str[1], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis2', 'volt', 'y_axis2', data_arrays[1], data_str[1])
            create_plot(data_str[2], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis3', 'speed', 'y_axis3', data_arrays[2], data_str[2])
            create_plot(data_str[3], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis4', 'current', 'y_axis4', data_arrays[3], data_str[3])
            create_plot(data_str[4], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis5', 'volt', 'y_axis5', data_arrays[4], data_str[4])
            create_plot(data_str[5], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis6', 'en_state', 'y_axis6', data_arrays[5], data_str[5])
            create_plot(data_str[6], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis7', 'state', 'y_axis7', data_arrays[6], data_str[6])
            create_plot(data_str[7], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis8', 'feedback', 'y_axis8', data_arrays[7], data_str[7])
            create_plot(data_str[8], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis9', 'torque', 'y_axis9', data_arrays[8], data_str[8])
            create_plot(data_str[9], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis10', 'torque_cmd', 'y_axis10', data_arrays[9], data_str[9])
            create_plot(data_str[10], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*5)), 'x_axis11', 'speed_cmd', 'y_axis11', data_arrays[10], data_str[10])
            create_plot(data_str[11], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*5)), 'x_axis12', 'mode_en', 'y_axis12', data_arrays[11], data_str[11])
            
        with dpg.tab(label='VCU WSS/MCM', tracked=True):
            create_plot(data_str[12], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis13', 'percent', 'y_axis13', data_arrays[12], data_str[12])
            create_plot(data_str[13], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis14', 'percent', 'y_axis14', data_arrays[13], data_str[13])
            create_plot(data_str[14], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis15', 'FL_S', 'y_axis15', data_arrays[14], data_str[14])
            create_plot(data_str[15], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis16', 'FL_S', 'y_axis16', data_arrays[15], data_str[15])
            create_plot(data_str[16], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis17', 'FL_S', 'y_axis17', data_arrays[16], data_str[16])
            create_plot(data_str[17], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis18', 'FL_S', 'y_axis18', data_arrays[17], data_str[17])
            create_plot(data_str[18], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis19', 'Sense_Lost', 'y_axis19', data_arrays[18], data_str[18])
            create_plot(data_str[19], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis20', 'volt', 'y_axis20', data_arrays[19], data_str[19])
            create_plot(data_str[20], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis21', 'regen_mode', 'y_axis21', data_arrays[20], data_str[20])
            create_plot(data_str[21], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis22', 'max_torque', 'y_axis22', data_arrays[21], data_str[21])
        
        with dpg.tab(label='VCU BMS', tracked=True):
            create_plot(data_str[24], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis25', 'cell_temp', 'y_axis25', data_arrays[24], data_str[24])
            create_plot(data_str[25], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis26', 'cell_volt', 'y_axis26', data_arrays[25], data_str[25])
            create_plot(data_str[26], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis27', 'cell_volt', 'y_axis27', data_arrays[26], data_str[26])
            create_plot(data_str[27], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis28', 'temp_dc', 'y_axis28', data_arrays[27], data_str[27])
        
        with dpg.tab(label='VCU Power Limit/Launch Control', tracked=True):
            create_plot(data_str[28], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis29', 'total_error', 'y_axis29', data_arrays[28], data_str[28])
            create_plot(data_str[29], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis30', 'proportional', 'y_axis30', data_arrays[29], data_str[29])
            create_plot(data_str[30], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis31', 'integral', 'y_axis31', data_arrays[30], data_str[30])
            create_plot(data_str[31], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis32', 'torque_cmd', 'y_axis32', data_arrays[31], data_str[31])
            create_plot(data_str[32], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis33', 'torque_cmd', 'y_axis33', data_arrays[32], data_str[32])
            create_plot(data_str[33], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis34', 'slip_ratio', 'y_axis34', data_arrays[33], data_str[33])
            create_plot(data_str[34], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis35', 'PID_output', 'y_axis35', data_arrays[34], data_str[34])
            create_plot(data_str[35], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis36', 'proportional', 'y_axis36', data_arrays[35], data_str[35])
            create_plot(data_str[36], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis37', 'integral', 'y_axis37', data_arrays[36], data_str[36])
            create_plot(data_str[37], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis38', 'total_error', 'y_axis38', data_arrays[37], data_str[37])

        with dpg.tab(label='BMS/Misc', tracked=True):
            create_plot(data_str[38], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis39', 'current_state', 'y_axis39', data_arrays[38], data_str[38])
            create_plot(data_str[39], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis40', 'pos_closed', 'y_axis40', data_arrays[39], data_str[39])
            create_plot(data_str[40], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis41', 'neg_closed', 'y_axis41', data_arrays[40], data_str[40])
            create_plot(data_str[41], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis42', 'pack_volt', 'y_axis42', data_arrays[41], data_str[41])
            create_plot(data_str[42], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis43', 'state_of_charge', 'y_axis43', data_arrays[42], data_str[42])
            create_plot(data_str[43], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis44', 'cell_temp', 'y_axis44', data_arrays[43], data_str[43])
            create_plot(data_str[44], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis45', 'humidity', 'y_axis45', data_arrays[44], data_str[44])
            create_plot(data_str[45], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis46', 'temp', 'y_axis46', data_arrays[45], data_str[45])
            create_plot(data_str[22], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis23', 'speed_kph', 'y_axis23', data_arrays[22], data_str[22])
            create_plot(data_str[23], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis24', 'angle', 'y_axis24', data_arrays[23], data_str[23])


#BUTTONS STUFF
def PL1(): udr.send_data("1")
def PL2(): udr.send_data("2")
def PL3(): udr.send_data("3")
def Regen1(): udr.send_data("4")
def Regen2(): udr.send_data("5")
def Eff1(): udr.send_data("6")
def Eff2(): udr.send_data("7")
def Eff3(): udr.send_data("8")

with dpg.window(label='BUTTONS', pos=(1010, 20), width=250, height=250):
    dpg.add_button(label="PL_Target_Power_1", callback=PL1)
    dpg.add_button(label="PL_Target_Power_2", callback=PL2)
    dpg.add_button(label="PL_Target_Power_3", callback=PL3)
    dpg.add_button(label="Regen_Mode_1", callback=Regen1)
    dpg.add_button(label="Regen_Mode_2", callback=Regen2)
    dpg.add_button(label="Efficiency_Mode_1", callback=Eff1)
    dpg.add_button(label="Efficiency_Mode_2", callback=Eff2)
    dpg.add_button(label="Efficiency_Mode_3", callback=Eff3)


#DATA COUNTER
with dpg.window(label='Data Counter', pos=(1260, 20), width=250, height=250):
    dpg.add_text(tag='COUNTER', default_value=str(5))

def data_count():
    while(True):
        dpg.set_value('COUNTER', str(udr.count))
        time.sleep(1.3)


#WARNING/ERROR LOG STUFF
with dpg.window(label='DATA LOG', tag="terminal", pos=(1010, 270), width=500, height=500):
    dpg.add_text('Terminal: ')
    dpg.add_child_window(tag='log_container', autosize_x=True, width=500, height=400, horizontal_scrollbar=True)

def thresh_check(data_type, value, time, type):
    # warning = 0, danger = 1
    if (type == 0): 
        msg = f"WARNING: {data_type} exceeded threshold at {time:.2f}s with {value}"
        dpg.add_text(msg, parent='log_container', color=WARN_COLOR) 
        dpg.set_y_scroll('log_container', 9999) #auto scroll function

    if (type == 1): 
        msg = f"DANGER: {data_type} exceeded threshold at {time:.2f}s with {value}"
        dpg.add_text(msg, parent='log_container', color=DANGER_COLOR) #orange tuple
        dpg.set_y_scroll('log_container', 9999) #auto scroll function

def power_calc(current, volt):
    power = current * volt
    msg = f"CURRENT POWER: {power}"
    dpg.add_text(msg, parent='log_container')
    dpg.set_y_scroll('log_container', 9999) #auto scroll function


#dpg.show_font_manager()

dpg.create_context()
dpg.create_viewport(title='TCS App', width=1400, height=1000)
dpg.setup_dearpygui()
dpg.show_viewport()

dpg.set_global_font_scale(1.0)

#list of threads
thread1 = threading.Thread(target=update_all)
thread2 = threading.Thread(target=data_count)
thread1.start()
thread2.start()

dpg.start_dearpygui()

dpg.destroy_context()
