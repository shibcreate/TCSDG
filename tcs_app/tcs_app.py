import dearpygui.dearpygui as dpg
import uart_data_read as udr

import time
import threading

nsamples = 1000
global time_x
time_x = [0.0]*nsamples
global em_cur
em_cur = [0.0]*nsamples
global em_volt
em_volt = [0.0]*nsamples

global pause_check 
pause_check = False

def update_all():
    t0 = time.time()
    while True:
        #if (pause_check == False):
            for i in range(11):
                udr.update_data()
            time_x.append(time.time() - t0) #update time x axis
            em_cur.append(udr.em_current[-1]) #update y axis
            em_volt.append(udr.em_volt[-1])

            #set series x and y to last nsamples
            dpg.set_value('series_tag1', [list(time_x[-nsamples:]), list(em_cur[-nsamples:])])
            dpg.set_value('series_tag2', [list(time_x[-nsamples:]), list(em_volt[-nsamples:])])
            dpg.fit_axis_data('x_axis')
            dpg.fit_axis_data('y_axis')
            dpg.fit_axis_data('x_axis1')
            dpg.fit_axis_data('y_axis2')

            #time.sleep(0.1)

dpg.create_context()
with dpg.window(label='Graphs', tag='win', width=1000, height = 800):
    
    with dpg.plot(label='em current vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='em current', tag='y_axis')

        dpg.add_line_series(x=list(time_x), y=list(em_cur), label='Temp', parent='y_axis', tag='series_tag1')
    
    with dpg.plot(label='em volt vs time', height=300, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='time', tag='x_axis1')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='em volt', tag='y_axis2')

        dpg.add_line_series(x=list(time_x), y=list(em_volt), label='Temp', parent='y_axis2', tag='series_tag2')

def pause_graph(sender, data):
    if (pause_check):
        pause_check = True
    else:
        pause_check = False


dpg.create_viewport(title='TCS App', width=1000, height=800)

dpg.setup_dearpygui()
dpg.show_viewport()

#dpg.add_button(label="Pause", callback=pause_graph)

thread = threading.Thread(target=update_all)
thread.start()
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