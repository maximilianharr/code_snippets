#-*- coding: utf-8 -*-

# Plot vehicle data

# Tkinter Imports
import Tkinter
from   Tkinter import *
from   tkFileDialog import askopenfilename
import tkMessageBox
# xml Imports
import xml.etree.ElementTree as ET
# matplotlib Imports
import matplotlib.pyplot as plt
from   matplotlib.figure import Figure
from   matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
import matplotlib
import numpy as np

matplotlib.use('TkAgg')

# Define global variables for user input
sensor_id = 0;
x_id = 0;
y_id = 1;
hold_on = 0;
my_colors = ["r","g","b"]

# Read XML file
# xml_name = '20160908_rtk_05_insignia.xml'
# tree = ET.parse(xml_name)
# root = tree.getroot()

# plot in window
top = Tkinter.Tk()
f = Figure(figsize=(8, 6), dpi=100)
canvas = FigureCanvasTkAgg(f, master=top)
toolbar = NavigationToolbar2TkAgg(canvas, top)

########################################################################
# Function for more Plots in the Graphicwindow 
def save_hold_on():
  global hold_on
  hold_on = abs(hold_on -1)

# Global val for x-axis-id to be plotted
def save_x_id(x_val):
  global x_id
  x_id = x_val

# Global val for y-axis-id to be plotted
def save_y_id(y_val):
  global y_id
  y_id = y_val

# 
def save_sensor_id(tag_id):
  
  # Store sensor id
  global sensor_id
  sensor_id = tag_id
  
  # Count sensor outputs
  sensor_string = root[sensor_id].text
  sensor_attr = sensor_string.split(';')
  sensor_size = sensor_attr[0].split(',')
  
  # make x/y menu for the selected sensor-ID
  x_axismenu.delete(0,END)
  y_axismenu.delete(0,END)
  for iter_x in range( len(sensor_size) ):
    x_axismenu.add_command(label=iter_x+1, command=lambda x_val=iter_x : save_x_id(x_val) )
    y_axismenu.add_command(label=iter_x+1, command=lambda y_val=iter_x : save_y_id(y_val) )
  
# Get the choosen xml-file and parse xml
def get_xml_name():
  global xml_name, root, tree
  Tk().withdraw()
  xml_name = askopenfilename()
  tree = ET.parse(xml_name)
  root = tree.getroot()
  
def print_plot():
  a = f.add_subplot(111)
 
  # Separate each line with ';'
  imu_str = root[sensor_id].text
  imu_attr = imu_str.split(";")
  
  # Init data lists
  x = [0]*(len(imu_attr)-1)
  y = [0]*(len(imu_attr)-1)
  imu_data = [0]*len(imu_attr)

  # Separate each value with ',' and store in data list
  for i in range(len(imu_attr)):
    imu_data[i] = imu_attr[i].split(",")

  # Store x and y values
  count_i = 0
  for i in range(len(imu_attr)-1):

    # Check if all sensor data (except time stamp) is zero (imu_data is string!)
    zero_val = 0;
    for k in range(len(imu_data[i])-2):
      if imu_data[i][k+1] != '0':
        zero_val = 1
    # Store sensor data
    if zero_val == 1:
      # Store time vector (subtract initial time stamp)
      if x_id == 0:
        x[count_i] = float(imu_data[i][x_id])-float(imu_data[0][x_id])
      else:
        x[count_i] = float(imu_data[i][x_id])
      # Store sensor data
      y[count_i] = float(imu_data[i][y_id])
      count_i = count_i+1
    else:
      # Remove last element from list
      del x[-1]
      del y[-1]      

  # Inform the user that some sensor data is invalid
  if (count_i+1 < len(imu_attr)):
    print('Warning, some sensor-data contains zero values [%d/%d]' % (len(imu_attr)-count_i-1, len(imu_attr) ))  

  # Plot data
  if hold_on==0:
    a.clear()
  color_id = min( 2, len(a.lines) )
  a.plot(x,y,'-', color=my_colors[color_id])
  a.grid()

  # Plot window 
  canvas.show()
  canvas.get_tk_widget().pack()

  # Toolbar 
  toolbar.update()
  canvas._tkcanvas.pack()

# Get "Close Window"-Event (WM_DELETE_WINDOW) with Tkinter protocol handlers
def on_closing():
  # if tkMessageBox.askokcancel("Quit", "Do you want to quit?"):
    top.destroy()
    sys.exit(0)
        

########################################################################

# Window title
top.title("Plot vehicle data")

# Window geometry
top.geometry("800x600")

# Options for Menu
menu = Menu(top)
top.config(menu=menu)
filemenu = Menu(menu)

# Make Senor Menu
get_xml_name()
menu.add_cascade(label="SENSOR", menu=filemenu)
for i in range( len(root.findall('*')) ):
  tag_name = root[i].tag
  filemenu.add_command(label=tag_name, command=lambda tag_id=i : save_sensor_id(tag_id) )

# Make X-Achse Menu
x_axismenu = Menu(menu)
menu.add_cascade(label="X-Achse", menu=x_axismenu) 

# Make Y-Achse Menu
y_axismenu = Menu(menu)
menu.add_cascade(label="Y-Achse", menu=y_axismenu)

# 3 Buttons in the headline
menu.add_radiobutton(label="Plot", command=print_plot)
menu.add_checkbutton(label="Hold on", command=save_hold_on)
menu.add_radiobutton(label="Choose file ..", command = get_xml_name)

# Link WM_DELETE_WINDOW (User presses close button) with function
top.protocol("WM_DELETE_WINDOW", on_closing)

# Loop main
top.mainloop()


