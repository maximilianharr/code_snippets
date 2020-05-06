#-*- coding: utf-8 -*-
import Tkinter
from Tkinter import *
import tkMessageBox
import proc_veh_data as pvd 
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

#def sensor_imu():
 # sensor_id = 0 
  #tkMessageBox.showinfo(" Sensor_id :", "sensor_id = 0") 
########################################################################
def plot_b1(): # Plot: X-Achse: time stamp / Y-Achse: accelerationLongitudinal
  x = 0
  y = 1
  sensor = 1
  vd.plot(sensor,y,x)
def plot_a1(): # Plot: X-Achse: accelerationLongitudinal / Y-Achse: time stamp
  x = 1
  y = 0
  sensor = 1
  pvd.plot(sensor,y,x)
def b():
  x_achsemenu2.delete(0,END)
  x_achsemenu2.add_command(label="time stamp",command=plot_b1)
def a():
  x_achsemenu2.delete(0,END)
  x_achsemenu2.add_command(label="accelerationLongitudinal",command=plot_a1)
def sensor_imu2():
  y_achsemenu.delete(0,END)
  y_achsemenu.add_command(label="time stamp",command=a)
  y_achsemenu.add_command(label="accelerationLongitudinal",command=b)
########################################################################
def plot_a2a(): # Plot: X-Achse: rateYawAngle / Y-Achse: time stamp
  x = 1
  y = 0
  sensor = 0
  pvd.plot(sensor,y,x)
def plot_a2b(): # Plot: X-Achse: accelerationLongitudinal / Y-Achse: time stamp
  x = 2
  y = 0
  sensor = 0
  pvd.plot(sensor,y,x)
def plot_b2a(): # Plot: X-Achse: time stamp / Y-Achse: rateYawAngle
  x = 0
  y = 1
  sensor = 0
  pvd.plot(sensor,y,x)
def plot_b2b(): # Plot: X-Achse: accelerationLongitudinal / Y-Achse: rateYawAngle 
  x = 2
  y = 1
  sensor = 0
  pvd.plot(sensor,y,x)
def plot_c2a(): # Plot: X-Achse: time stamp / Y-Achse: accelerationLongitudinal
  x = 0
  y = 2
  sensor = 0
  pvd.plot(sensor,y,x)
def plot_c2b(): # Plot: X-Achse: rateYawAngle / Y-Achse: accelerationLongitudinal
  x = 1
  y = 2
  sensor = 0
  pvd.plot(sensor,y,x)
def c1():
  x_achsemenu2.delete(0,END)
  x_achsemenu2.add_command(label="time stamp",command=plot_c2a)
  x_achsemenu2.add_command(label="rateYawAngle",command=plot_c2b)
def b1():
  x_achsemenu2.delete(0,END)
  x_achsemenu2.add_command(label="time stamp",command=plot_b2a)
  x_achsemenu2.add_command(label="accelerationLongitudinal",command=plot_b2b)
def a1():
  x_achsemenu2.delete(0,END)
  x_achsemenu2.add_command(label="rateYawAngle",command=plot_a2a)
  x_achsemenu2.add_command(label="accelerationLongitudinal",command=plot_a2b)
def sensor_imu():
  y_achsemenu.delete(0,END)
  y_achsemenu.add_command(label="time stamp",command=a1)
  y_achsemenu.add_command(label="rateYawAngle",command=b1)
  y_achsemenu.add_command(label="accelerationLongitudinal",command=c1)
########################################################################


top = Tkinter.Tk()

top.title("Plot vehicle data")
top.geometry("400x400")
menu = Menu(top)
top.config(menu=menu)
filemenu = Menu(menu)

menu.add_cascade(label="SENSOR", menu=filemenu)
filemenu.add_command(label="imu", command=sensor_imu)
filemenu.add_command(label="imu2", command=sensor_imu2)


y_achsemenu = Menu(menu)
menu.add_cascade(label="Y-Achse", menu=y_achsemenu)


x_achsemenu2 = Menu(menu)
menu.add_cascade(label="X-Achse", menu=x_achsemenu2) 


top.mainloop()
