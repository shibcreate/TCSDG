import dearpygui.dearpygui as dpg
import uart_data_read as udr

import time
import threading


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


#INIT FOR DATA GRAPHS
nsamples = 5
DATA_LEN = len(data_str)
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
    global data_arrays
    t_u = time.time()
    while True:
        for i in range(10):
            udr.update_data()
            # data_arrays[i] = udr.data_arrays[i]
        
        time_x.append(time.time() - t_u) #update time x axis
        em_curr.append(udr.em_curr[-1])
        em_volt.append(udr.em_volt[-1])
        motor_speed.append(udr.motor_speed[-1])
        bus_curr.append(udr.bus_curr[-1])
        bus_volt.append(udr.bus_volt[-1])
        inv_en_state.append(udr.inv_en_state[-1])
        inv_state.append(udr.inv_state[-1])
        torq_feed.append(udr.torq_feed[-1])
        cmded_torque.append(udr.cmded_torque[-1])
        torq_cmd.append(udr.torq_cmd[-1])
        speed_cmd.append(udr.speed_cmd[-1])
        speed_mode_en.append(udr.speed_mode_en[-1])
        throttle_perc.append(udr.throttle_perc[-1])
        brake_perc.append(udr.brake_perc[-1])
        vcu_fl.append(udr.vcu_fl[-1])
        vcu_fr.append(udr.vcu_fr[-1])
        vcu_rl.append(udr.vcu_rl[-1])
        vcu_rr.append(udr.vcu_rr[-1])
        term_sense_lost.append(udr.term_sense_lost[-1])
        batt_low_volt.append(udr.batt_low_volt[-1])
        regen_mode.append(udr.regen_mode[-1])
        regen_max_torq.append(udr.regen_max_torq[-1])
        speed_kph.append(udr.speed_kph[-1])
        steer_angle.append(udr.steer_angle[-1])
        high_cell_temp.append(udr.high_cell_temp[-1])
        high_cell_volt.append(udr.high_cell_volt[-1])
        low_cell_volt.append(udr.low_cell_volt[-1])
        high_cell_temp_dc.append(udr.high_cell_temp_dc[-1])
        PL_total_err.append(udr.PL_total_err[-1])
        PL_prop.append(udr.PL_prop[-1])
        PL_integral.append(udr.PL_integral[-1])
        PL_torq_cmd.append(udr.PL_torq_cmd[-1])
        LC_torq_cmd.append(udr.LC_torq_cmd[-1])
        LC_slip.append(udr.LC_slip[-1])
        LC_pid.append(udr.LC_pid[-1])
        LC_prop.append(udr.LC_prop[-1])
        LC_integral.append(udr.LC_integral[-1])
        LC_total_err.append(udr.LC_total_err[-1])
        BMS_state.append(udr.BMS_state[-1])
        BMS_main_pos.append(udr.BMS_main_pos[-1])
        BMS_main_neg.append(udr.BMS_main_neg[-1])
        pack_volt.append(udr.pack_volt[-1])
        BMS_charge.append(udr.BMS_charge[-1])
        BMS_high_temp.append(udr.BMS_high_temp[-1])
        humidity.append(udr.humidity[-1])
        temp.append(udr.temp[-1])

        for i in range(DATA_LEN):
            dpg.set_value(data_str[i], [list(time_x), list(data_arrays[i])])
            dpg.fit_axis_data(f'x_axis{i+1}')
            dpg.fit_axis_data(f'y_axis{i+1}')

        #threshold checks
        if (pack_volt[-1] < thresholds[0] and pack_volt[-1] > thresholds[1]):
            thresh_check("pack voltage", pack_volt[-1], time_x[-1], 0)
        if (pack_volt[-1] < thresholds[1]):
            thresh_check("pack voltage", pack_volt[-1], time_x[-1], 1)
        if (cmded_torque[-1] >= thresholds[2]):
            thresh_check("commanded torque", cmded_torque[-1], time_x[-1], 1)      
        if (throttle_perc[-1] <= thresholds[3]):
            thresh_check("throttle percent", throttle_perc[-1], time_x[-1], 1)

        #error check
        faults = udr.faults
        for i in range(FAULT_LEN):
            if(faults[i] > 0):
                print(faults)
                msg = f"FAULT OCCURRED OF TYPE: {fault_str[i]}"
                dpg.add_text(msg, parent='log_container', color=FAULT_COLOR) #red tuple
                dpg.set_y_scroll('log_container', 9999) #auto scroll function
                faults[i] = 0
                udr.faults[i] = 0

        #power calculations: dc_bus_current * dc_voltage
        power_calc(bus_curr[-1], bus_volt[-1])


dpg.create_context()


#GRAPHS WINDOWS
def create_plot(plot_label, pos_coord, xtag, ylabel, ytag, ylist, line_tag):
    with dpg.plot(label=plot_label, pos=pos_coord, height=GRAPH_HEIGHT, width=GRAPH_WIDTH, crosshairs=True):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='Time (s)', tag=xtag)
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label=ylabel, tag=ytag)
        dpg.add_line_series(x=list(time_x), y=list(ylist), parent=ytag, tag=line_tag)


with dpg.window(label='GRAPHS', tag='win', width=1000, height = 800, no_scroll_with_mouse=False):
    with dpg.tab_bar(label='tab_bar', reorderable=True):
        with dpg.tab(label='MCM', tracked=True):
            create_plot(data_str[0], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis1', 'current', 'y_axis1', em_curr, data_str[0])
            create_plot(data_str[1], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis2', 'volt', 'y_axis2', em_volt, data_str[1])
            create_plot(data_str[2], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis3', 'speed', 'y_axis3', motor_speed, data_str[2])
            create_plot(data_str[3], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis4', 'current', 'y_axis4', bus_curr, data_str[3])
            create_plot(data_str[4], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis5', 'volt', 'y_axis5', bus_volt, data_str[4])
            create_plot(data_str[5], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis6', 'en_state', 'y_axis6', inv_en_state, data_str[5])
            create_plot(data_str[6], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis7', 'state', 'y_axis7', inv_state, data_str[6])
            create_plot(data_str[7], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis8', 'feedback', 'y_axis8', torq_feed, data_str[7])
            create_plot(data_str[8], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis9', 'torque', 'y_axis9', cmded_torque, data_str[8])
            create_plot(data_str[9], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis10', 'torque_cmd', 'y_axis10', torq_cmd, data_str[9])
            create_plot(data_str[10], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*5)), 'x_axis11', 'speed_cmd', 'y_axis11', speed_cmd, data_str[10])
            create_plot(data_str[11], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*5)), 'x_axis12', 'mode_en', 'y_axis12', speed_mode_en, data_str[11])
            
        with dpg.tab(label='VCU WSS/MCM', tracked=True):
            create_plot(data_str[12], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis13', 'percent', 'y_axis13', throttle_perc, data_str[12])
            create_plot(data_str[13], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis14', 'percent', 'y_axis14', brake_perc, data_str[13])
            create_plot(data_str[14], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis15', 'FL_S', 'y_axis15', vcu_fl, data_str[14])
            create_plot(data_str[15], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis16', 'FL_S', 'y_axis16', vcu_fr, data_str[15])
            create_plot(data_str[16], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis17', 'FL_S', 'y_axis17', vcu_rl, data_str[16])
            create_plot(data_str[17], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis18', 'FL_S', 'y_axis18', vcu_rr, data_str[17])
            create_plot(data_str[18], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis19', 'Sense_Lost', 'y_axis19', term_sense_lost, data_str[18])
            create_plot(data_str[19], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis20', 'volt', 'y_axis20', batt_low_volt, data_str[19])
            create_plot(data_str[20], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis21', 'regen_mode', 'y_axis21', regen_mode, data_str[20])
            create_plot(data_str[21], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis22', 'max_torque', 'y_axis22', regen_max_torq, data_str[21])
        
        with dpg.tab(label='VCU BMS', tracked=True):
            create_plot(data_str[24], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis25', 'cell_temp', 'y_axis25', high_cell_temp, data_str[24])
            create_plot(data_str[25], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis26', 'cell_volt', 'y_axis26', high_cell_volt, data_str[25])
            create_plot(data_str[26], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis27', 'cell_volt', 'y_axis27', low_cell_volt, data_str[26])
            create_plot(data_str[27], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis28', 'temp_dc', 'y_axis28', high_cell_temp_dc, data_str[27])
        
        with dpg.tab(label='VCU Power Limit/Launch Control', tracked=True):
            create_plot(data_str[28], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis29', 'total_error', 'y_axis29', PL_total_err, data_str[28])
            create_plot(data_str[29], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis30', 'proportional', 'y_axis30', PL_prop, data_str[29])
            create_plot(data_str[30], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis31', 'integral', 'y_axis31', PL_integral, data_str[30])
            create_plot(data_str[31], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis32', 'torque_cmd', 'y_axis32', PL_torq_cmd, data_str[31])
            create_plot(data_str[32], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis33', 'torque_cmd', 'y_axis33', LC_torq_cmd, data_str[32])
            create_plot(data_str[33], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis34', 'slip_ratio', 'y_axis34', LC_slip, data_str[33])
            create_plot(data_str[34], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis35', 'PID_output', 'y_axis35', LC_pid, data_str[34])
            create_plot(data_str[35], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis36', 'proportional', 'y_axis36', LC_prop, data_str[35])
            create_plot(data_str[36], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis37', 'integral', 'y_axis37', LC_integral, data_str[36])
            create_plot(data_str[37], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis38', 'total_error', 'y_axis38', LC_total_err, data_str[37])

        with dpg.tab(label='BMS/Misc', tracked=True):
            create_plot(data_str[38], (GRAPH_POS_XL, GRAPH_POS_Y), 'x_axis39', 'current_state', 'y_axis39', BMS_state, data_str[38])
            create_plot(data_str[39], (GRAPH_POS_XR, GRAPH_POS_Y), 'x_axis40', 'pos_closed', 'y_axis40', BMS_main_pos, data_str[39])
            create_plot(data_str[40], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis41', 'neg_closed', 'y_axis41', BMS_main_neg, data_str[40])
            create_plot(data_str[41], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*1)), 'x_axis42', 'pack_volt', 'y_axis42', pack_volt, data_str[41])
            create_plot(data_str[42], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis43', 'state_of_charge', 'y_axis43', BMS_charge, data_str[42])
            create_plot(data_str[43], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*2)), 'x_axis44', 'cell_temp', 'y_axis44', BMS_high_temp, data_str[43])
            create_plot(data_str[44], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis45', 'humidity', 'y_axis45', humidity, data_str[44])
            create_plot(data_str[45], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*3)), 'x_axis46', 'temp', 'y_axis46', temp, data_str[45])
            create_plot(data_str[22], (GRAPH_POS_XL, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis23', 'speed_kph', 'y_axis23', speed_kph, data_str[22])
            create_plot(data_str[23], (GRAPH_POS_XR, GRAPH_POS_Y+(GRAPH_HEIGHT*4)), 'x_axis24', 'angle', 'y_axis24', steer_angle, data_str[23])


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
        time.sleep(1.5)


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
    msg = f"CURRENT POWER: {power:.3f}"
    dpg.add_text(msg, parent='log_container')
    dpg.set_y_scroll('log_container', 9999) #auto scroll function



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
