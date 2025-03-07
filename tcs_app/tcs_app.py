import dearpygui.dearpygui as dpg
import uart_data_read as udr

import time
import threading

nsamples = 10
global time_x #y_axis for most graphs
time_x = []
time_x.append(0)

def update_all():
    t0 = time.time()
    while True:
        udr.update_data() #update x axis
        time_x.append(time.time() - t0) #update time y axis

        #set series x and y to last nsamples
        dpg.set_value('series_tag', [list(udr.ground[-nsamples:]), list(time_x[-nsamples:])])
        dpg.fit_axis_data('x_axis')
        dpg.fit_axis_data('y_axis')

dpg.create_context()
with dpg.window(label='Tutorial', tag='win', width=800, height = 600):
    
    with dpg.plot(label='Ground Speed v Time', height=600, width=800):
        #create x and y axes, set to auto scale
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='ground speed', tag='x_axis')
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='time', tag='y_axis')

        dpg.add_line_series(udr.ground, time_x, label='Temp', parent='y_axis', tag='series_tag')

dpg.create_viewport(title='Custom Title', width=850, height=640)

dpg.setup_dearpygui()
dpg.show_viewport()

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