#-*- coding: utf-8 -*-
import Tkinter
from Tkinter import *
import tkMessageBox
import proc_veh_data as pvd 
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

top = Tkinter.Tk()

top.title("Plot vehicle data")


LA = Label(top, text="Bitte imu etc... wählen :")
LA.pack(side = LEFT)
eingabe = Entry(top, bd =5)
eingabe.pack( side = LEFT)

LA4 = Label(top, text="Bitte 2. imu etc... wählen :")
LA4.pack(side = RIGHT)
eingabe4 = Entry(top, bd =5)
eingabe4.pack(side = RIGHT)

LA2 = Label(top, text="Bitte die Y-Achse wählen :")
LA2.pack(side = LEFT)
eingabe2 = Entry(top, bd =5)
eingabe2.pack( side = LEFT)

LA5 = Label(top, text="Bitte die 2. Y-Achse wählen :")
LA5.pack(side = RIGHT)
eingabe5 = Entry(top, bd =5)
eingabe5.pack(side = RIGHT)

LA3 = Label(top, text="Bitte die X-Achse wählen :")
LA3.pack(side = LEFT)
eingabe3 = Entry(top, bd =5)
eingabe3.pack( side = LEFT)

LA6 = Label(top, text="Bitte die 2. X-Achse wählen :")
LA6.pack(side = RIGHT)
eingabe6 = Entry(top, bd =5)
eingabe6.pack(side = RIGHT)
def start1():
  sensor_id = int(eingabe.get())
  y_id = int(eingabe2.get())
  x_id = int(eingabe3.get())

  
    
  
  pvd.plot(sensor_id, y_id, x_id)
  return
B = Tkinter.Button(top, text ="Hier drüken zum Starten", command = start1)
B.pack()

def start2():
  sensor = int(eingabe.get())
  y = int(eingabe.get2())
  x = int(eingabe.get3())


  pvd.plot(sensor, y, x)
  return
C = Tkinter.Button(top, text ="Hier drücken für 2. Plot", command = start2)
C.pack()

top.mainloop()
