import dearpygui.dearpygui as dpg
import uart_data_read as udr
import uart_data_write as udw

import time
import threading

#init data for dpg graphs
nsamples = 100
global time_x, em_curr, em_volt, motor_speed, bus_current, torque_feed, command_torque, throttle_perc, steer_angle, pack_volt, volt_info, ground

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

global pause_check 
pause_check = False

def refresh():
    #global decarations of all data
    global time_x, em_curr, em_volt, motor_speed, bus_current, torque_feed, command_torque, throttle_perc, steer_angle, pack_volt, volt_info, ground

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
    t1 = time.time()
    while True:
        t_r = time.time() - t1
        #print(t_r)
        if (t_r > (60)):
            refresh()
            t1 = time.time()
            t_r = 0
               

def update_all():
    t_u = time.time()
    while True:
        #if (pause_check == False):
            for i in range(11):
                udr.update_data()
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

            #set series x and y to last nsamples
            dpg.set_value('series_tag1', [list(time_x), list(em_curr)])
            dpg.set_value('series_tag2', [list(time_x), list(em_volt)])
            dpg.set_value('series_tag3', [list(time_x), list(motor_speed)])
            dpg.set_value('series_tag4', [list(time_x), list(bus_current)])
            dpg.set_value('series_tag5', [list(time_x), list(torque_feed)])
            dpg.set_value('series_tag6', [list(time_x), list(command_torque)])
            dpg.set_value('series_tag7', [list(time_x), list(throttle_perc)])
            dpg.set_value('series_tag8', [list(time_x), list(steer_angle)])
            dpg.set_value('series_tag9', [list(time_x), list(pack_volt)])
            dpg.set_value('series_tag10', [list(time_x), list(volt_info)])
            dpg.set_value('series_tag11', [list(time_x), list(ground)])
            dpg.fit_axis_data('x_axis')
            dpg.fit_axis_data('y_axis')
            dpg.fit_axis_data('x_axis1')
            dpg.fit_axis_data('y_axis1')
            dpg.fit_axis_data('x_axis2')
            dpg.fit_axis_data('y_axis2')
            dpg.fit_axis_data('x_axis3')
            dpg.fit_axis_data('y_axis3')
            dpg.fit_axis_data('x_axis4')
            dpg.fit_axis_data('y_axis4')
            dpg.fit_axis_data('x_axis5')
            dpg.fit_axis_data('y_axis5')
            dpg.fit_axis_data('x_axis6')
            dpg.fit_axis_data('y_axis6')
            dpg.fit_axis_data('x_axis7')
            dpg.fit_axis_data('y_axis7')
            dpg.fit_axis_data('x_axis8')
            dpg.fit_axis_data('y_axis8')
            dpg.fit_axis_data('x_axis9')
            dpg.fit_axis_data('y_axis9')
            dpg.fit_axis_data('x_axis10')
            dpg.fit_axis_data('y_axis10')

            #time.sleep(0.1)


dpg.create_context()

#window with graphs
with dpg.window(label='Graphs', tag='win', width=1980, height = 1080):
    
    with dpg.plot(label='em current vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='em current', tag='y_axis')

        dpg.add_line_series(x=list(time_x), y=list(em_curr), label='Temp', parent='y_axis', tag='series_tag1')
    
    with dpg.plot(label='em volt vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis1')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='em volt', tag='y_axis1')

        dpg.add_line_series(x=list(time_x), y=list(em_volt), label='Temp', parent='y_axis1', tag='series_tag2')

    with dpg.plot(label='motor speed vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis2')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='motor speed', tag='y_axis2')

        dpg.add_line_series(x=list(time_x), y=list(motor_speed), label='Temp', parent='y_axis2', tag='series_tag3')

    with dpg.plot(label='bus current vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis3')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='bus current', tag='y_axis3')

        dpg.add_line_series(x=list(time_x), y=list(bus_current), label='Temp', parent='y_axis3', tag='series_tag4')
    
    with dpg.plot(label='torque feed vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis4')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='torque feed', tag='y_axis4')

        dpg.add_line_series(x=list(time_x), y=list(torque_feed), label='Temp', parent='y_axis4', tag='series_tag5')
    
    with dpg.plot(label='command torque vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis5')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='command torque', tag='y_axis5')

        dpg.add_line_series(x=list(time_x), y=list(command_torque), label='Temp', parent='y_axis5', tag='series_tag6')
    
    with dpg.plot(label='throttle perc vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis6')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='throttle perc', tag='y_axis6')

        dpg.add_line_series(x=list(time_x), y=list(throttle_perc), label='Temp', parent='y_axis6', tag='series_tag7')

    with dpg.plot(label='steer angle vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis7')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='steer angle', tag='y_axis7')

        dpg.add_line_series(x=list(time_x), y=list(steer_angle), label='Temp', parent='y_axis7', tag='series_tag8')

    with dpg.plot(label='pack volt vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis8')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='pack volt', tag='y_axis8')

        dpg.add_line_series(x=list(time_x), y=list(pack_volt), label='Temp', parent='y_axis8', tag='series_tag9')

    with dpg.plot(label='volt info vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis9')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='volt info', tag='y_axis9')

        dpg.add_line_series(x=list(time_x), y=list(volt_info), label='Temp', parent='y_axis9', tag='series_tag10')

    with dpg.plot(label='ground vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis10')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='ground', tag='y_axis10')

        dpg.add_line_series(x=list(time_x), y=list(ground), label='Temp', parent='y_axis10', tag='series_tag11')


#window with buttons
def call_P1():
    udw.send_data("1")
def call_P2():
    udw.send_data("2")
def call_P3():
    udw.send_data("3")
def call_P4():
    udw.send_data("4")
def call_P5():
    udw.send_data("5")
def call_P6():
    udw.send_data("6")
def call_P7():
    udw.send_data("7")
def call_P8():
    udw.send_data("8")
def call_P9():
    udw.send_data("9")


with dpg.window(label='Buttons'):
    dpg.add_button(label="P1", callback=call_P1)
    dpg.add_button(label="P2", callback=call_P2)
    dpg.add_button(label="P3", callback=call_P3)
    dpg.add_button(label="P4", callback=call_P4)
    dpg.add_button(label="P5", callback=call_P5)
    dpg.add_button(label="P6", callback=call_P6)
    dpg.add_button(label="P7", callback=call_P7)
    dpg.add_button(label="P8", callback=call_P8)
    dpg.add_button(label="P9", callback=call_P9)


def pause_graph(sender, data):
    if (pause_check):
        pause_check = True
    else:
        pause_check = False


dpg.create_viewport(title='TCS App', width=1000, height=800)

dpg.setup_dearpygui()
dpg.show_viewport()

#dpg.add_button(label="Pause", callback=pause_graph)

#list of threads
thread1 = threading.Thread(target=update_all)
thread2 = threading.Thread(target=refresh_check)
thread1.start()
thread2.start()
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