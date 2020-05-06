# -*- coding: utf-8 -*-
# @todo Eingabe einlesen > und ausgeben
# @todo Plot starten mit Knopf und Sinuskurve plotten

#Für die Sinus-und Cosinus Kurve
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#Für das GUI Programm
import Tkinter 
import tkMessageBox
from Tkinter import Entry
from Tkinter import RIGHT
from Tkinter import LEFT
from Tkinter import Label
top = Tkinter.Tk()#Erzeugung eines Fensters

top.title("GUI Programm")
top.geometry("500x600")

def Hallo(): # Funktion 1
   tkMessageBox.showinfo( "Ich hab es Geschafft!", "Was hast du Geschafft??")
   tkMessageBox.geometry("300x400")

B = Tkinter.Button(top, text ="Klick mich!", command = Hallo)
B.pack()

def Prank(): # Funktion 2
   tkMessageBox.showinfo( "Dich zu", "PRANKEN!!!")
   tkMessageBox.geometry("500x600")

C = Tkinter.Button(top, text ="Kilck mich um es zu erfahren!", command = Prank)
C.pack()

# Funtktion 4
LA = Label(top, text="Wie fühlst du dich?\nHier eingeben!")
LA.pack(side = LEFT)
eingabe = Entry(top, bd =5)
eingabe.pack(side = RIGHT)

def Lesen():
    
    tkMessageBox.showinfo( "Du fühlst dich", (eingabe.get()))
    tkMessageBox.geometry("1000x1000")


knopf3 = Tkinter.Button(top, text ="Hier drücken zum bestätigen!", command = Lesen)
knopf3.pack()

LB = Label(top, text="Bitte beachten die Legende\nsowie Beschriftung\nsteht im Python")
LB.pack()
#Sinus Kurve
def Sinus():
  print ('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
  print ('~~~~~~~~~~~~~~~~~~~~~~~Legende~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
  print ('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
  print
  print
  print ('~~~~~~~~~~Sinus hat die Farbe~~~~~GRÜN~~~~~~~~~~~~~~~~~~~~')
  print
  print ('~~~~~~~~~~Cosinus hat die Farbe~~~~ROT~~~~~~~~~~~~~~~~~~~~')
  print
  print
  print ('~~~~~~~~~X-Achse ist Waagrecht~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
  print
  print ('~~~~~~~~~Y-Achse ist Senkrecht~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

  fig, ax = plt.subplots()
  #line, = ax.plot(np.random.rand(10))
  ax.set_ylim(-1, 1)
  ax.set_xlim(0,2*np.pi)

  def f(x, a=1, b=0):
    return a*np.sin(x) + b*np.cos(x)
  x = np.linspace(0, 2 * np.pi, 500)
  y = f(x)
  plt.plot(x, y, 'gx', x, f(x, 1, 0), 'g')
  plt.plot(x, y, 'gx', x, f(x, 0, 1), 'r')

  plt.show()

knopf4 = Tkinter.Button(top, text ="Hier öffnet man die Sinus Kurve.",command = Sinus)
knopf4.pack()

top.mainloop()
