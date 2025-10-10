import dearpygui.dearpygui as dpg
import uart_data_read as udr

import time
import threading
from queue import Queue

#init data for dpg graphs
nsamples = 100
time_x = [0.0]*nsamples
em_curr = [0.0]*nsamples
em_volt = [0.0]*nsamples
motor_speed = [0.0]*nsamples
bus_current = [0.0]*nsamples
torque_feed = [0.0]*nsamples
command_torque = [0.0]*nsamples
throttle_perc = [0.0]*nsamples
steer_angle = [0.0]*nsamples
pack_volt = [0.0]*nsamples  
volt_info = [0.0]*nsamples
ground = [0.0]*nsamples

global count
count = 0

global pause
pause = False

data_q = Queue()

#for warning/error functions, easy to change if needed
global thresholds
thresholds = [70, 20, 30]

#indices for thresholds:
# 0 - em volt
# 1 - motor speed
# ...

def refresh():
    #global decarations of all data
    global time_x, em_curr, em_volt, motor_speed, bus_current, torque_feed, command_torque, throttle_perc, steer_angle
    global pack_volt, volt_info, ground

    time_x = [0.0]*nsamples
    em_curr = [0.0]*nsamples
    em_volt = [0.0]*nsamples
    motor_speed = [0.0]*nsamples
    bus_current = [0.0]*nsamples
    torque_feed = [0.0]*nsamples
    command_torque = [0.0]*nsamples
    throttle_perc = [0.0]*nsamples
    steer_angle = [0.0]*nsamples
    pack_volt = [0.0]*nsamples  
    volt_info = [0.0]*nsamples
    ground = [0.0]*nsamples


def refresh_check():
    while True:
        if (count == 50):
            refresh()
    '''
    t1 = time.time()
    while True:
        t_r = time.time() - t1
        #print(t_r)
        if (t_r > (60)):
            refresh()
            t1 = time.time()
            t_r = 0
    '''
               
refresh()

#MAIN THREAD
def update_all():
    global count
    count = 0
    t_u = time.time()
    while True:
        #if (pause_check == False):
        for i in range(11):
            udr.update_data()
            count = count + 1
            print(count)
        time_x.append(time.time() - t_u) #update time x axis
        em_curr.append(udr.em_current[-1]) #update y axis
        em_volt.append(udr.em_volt[-1])
        motor_speed.append(udr.motor_speed[-1])
        bus_current.append(udr.bus_current[-1])
        torque_feed.append(udr.torque_feed[-1])
        command_torque.append(udr.command_torque[-1])
        throttle_perc.append(udr.throttle_perc[-1])
        steer_angle.append(udr.steer_angle[-1])
        pack_volt.append(udr.pack_volt[-1])
        volt_info.append(udr.volt_info[-1])
        ground.append(udr.ground[-1])


        #threshold checks
        if (em_volt[-1] >= thresholds[0]):
            thresh_check("em_volt", em_volt[-1], time_x[-1])


        #occasional refresh
        #if (count == 5000):
        #    refresh()
        #    count = 0

        #set series x and y to last nsamples
        dpg.set_value('tag_em_curr', [list(time_x), list(em_curr)])
        dpg.set_value('tag_em_volt', [list(time_x), list(em_volt)])
        dpg.set_value('tag_mspeed', [list(time_x), list(motor_speed)])
        dpg.set_value('tag_bcurr', [list(time_x), list(bus_current)])
        dpg.set_value('tag_tfeed', [list(time_x), list(torque_feed)])
        dpg.set_value('tag_tcmd', [list(time_x), list(command_torque)])
        dpg.set_value('tag_throttle', [list(time_x), list(throttle_perc)])
        dpg.set_value('tag_steer', [list(time_x), list(steer_angle)])
        dpg.set_value('tag_pvolt', [list(time_x), list(pack_volt)])
        dpg.set_value('tag_volt_info', [list(time_x), list(volt_info)])
        dpg.set_value('tag_ground', [list(time_x), list(ground)])

        # fit axes
        for i in range(11):
            dpg.fit_axis_data(f'x_axis{i+1}')
            dpg.fit_axis_data(f'y_axis{i+1}')

        #time.sleep(0.1)

dpg.create_context()


#GRAPHS WINDOWS
with dpg.window(label='GRAPHS', tag='win', width=1980, height = 1080):
    
    with dpg.plot(label='em current vs time', pos=(0, 20), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis1')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='em current', tag='y_axis1')

        dpg.add_line_series(x=list(time_x), y=list(em_curr), label='Temp', parent='y_axis1', tag='tag_em_curr')
    
    with dpg.plot(label='em volt vs time', pos=(600, 20), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis2')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='em volt', tag='y_axis2')

        dpg.add_line_series(x=list(time_x), y=list(em_volt), label='Temp', parent='y_axis2', tag='tag_em_volt')

    with dpg.plot(label='motor speed vs time', pos=(0, 270), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis3')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='motor speed', tag='y_axis3')

        dpg.add_line_series(x=list(time_x), y=list(motor_speed), label='Temp', parent='y_axis3', tag='tag_mspeed')

    with dpg.plot(label='bus current vs time', pos=(600, 270), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis4')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='bus current', tag='y_axis4')

        dpg.add_line_series(x=list(time_x), y=list(bus_current), label='Temp', parent='y_axis4', tag='tag_bcurr')
    
    with dpg.plot(label='torque feed vs time', pos=(0, 520), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis5')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='torque feed', tag='y_axis5')

        dpg.add_line_series(x=list(time_x), y=list(torque_feed), label='Temp', parent='y_axis5', tag='tag_tfeed')
    
    with dpg.plot(label='command torque vs time', pos=(600, 520), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis6')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='command torque', tag='y_axis6')

        dpg.add_line_series(x=list(time_x), y=list(command_torque), label='Temp', parent='y_axis6', tag='tag_tcmd')
    
    with dpg.plot(label='throttle perc vs time', pos=(0, 770), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis7')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='throttle perc', tag='y_axis7')

        dpg.add_line_series(x=list(time_x), y=list(throttle_perc), label='Temp', parent='y_axis7', tag='tag_throttle')

    with dpg.plot(label='steer angle vs time', pos=(600, 770), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis8')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='steer angle', tag='y_axis8')

        dpg.add_line_series(x=list(time_x), y=list(steer_angle), label='Temp', parent='y_axis8', tag='tag_steer')

    with dpg.plot(label='pack volt vs time', pos=(0, 1020), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis9')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='pack volt', tag='y_axis9')

        dpg.add_line_series(x=list(time_x), y=list(pack_volt), label='Temp', parent='y_axis9', tag='tag_pvolt')

    with dpg.plot(label='volt info vs time', pos=(600, 1020), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis10')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='volt info', tag='y_axis10')

        dpg.add_line_series(x=list(time_x), y=list(volt_info), label='Temp', parent='y_axis10', tag='tag_volt_info')

    with dpg.plot(label='ground vs time', pos=(0, 1270), height=250, width=600):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis11')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='ground', tag='y_axis11')

        dpg.add_line_series(x=list(time_x), y=list(ground), label='Temp', parent='y_axis11', tag='tag_ground')


#BUTTONS STUFF
def call_P1(): udr.send_data("1")
def call_P2(): udr.send_data("2")
def call_P3(): udr.send_data("3")
def call_P4(): udr.send_data("4")
def call_P5(): udr.send_data("5")
def call_P6(): udr.send_data("6")
def call_P7(): udr.send_data("7")
def call_P8(): udr.send_data("8")
def call_P9(): udr.send_data("9")

with dpg.window(label='BUTTONS', pos=(1600, 300), width=150, height=200):
    for i in range(1, 10):
        dpg.add_button(label=f"P{i}", callback=globals()[f'call_P{i}'])


#WARNING/ERROR LOG STUFF
with dpg.window(label='DATA LOG', tag="terminal", pos=(1600, 20), width=150, height=250):
    dpg.add_text('Terminal: ')
    dpg.add_child_window(tag='log_container', autosize_x=True, height=250, horizontal_scrollbar=True)

def thresh_check(data_type, value, time):
    msg = f"WARNING: {data_type} exceeded threshold at {time:.2f}s with {value}"
    dpg.add_text(msg, parent='log_container')
    dpg.set_y_scroll('log_container', 9999) #auto scroll function


def pause_graph(sender, data):
    if (pause_check):
        pause_check = True
    else:
        pause_check = False

#dpg.show_font_manager()

dpg.create_context()
dpg.create_viewport(title='TCS App', width=1000, height=600)
dpg.setup_dearpygui()
dpg.show_viewport()

dpg.set_global_font_scale(1.0)
#dpg.add_button(label="Pause", callback=pause_graph)

#list of threads
thread1 = threading.Thread(target=update_all)
thread2 = threading.Thread(target=refresh_check)
thread1.start()
#thread2.start()
#thread3.start()

dpg.start_dearpygui()

dpg.destroy_context()

'''
# example dpg setup
def save_callback():
    print("Save Clicked")

dpg.create_context()
dpg.create_viewport()
dpg.setup_dearpygui()

with dpg.window(label="Example window"):
    dpg.add_text("Hello world")
    dpg.add_button(label="Save", callback=save_callback)
    dpg.add_input_text(label="string")
    dpg.add_slider_float(label="float")

dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
'''