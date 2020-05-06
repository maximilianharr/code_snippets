#-*- coding: utf-8 -*-
import Tkinter
from Tkinter import *
import tkMessageBox
import proc_veh_data as pvd 
import numpy as np
import matplotlib.pyplot as plt

root = Tk()
root.title("Chatten mit Python")

text_frame = Frame(root)

textfield = Text(text_frame, bg="light blue")
textfield.pack(side=LEFT)

scrollbar = Scrollbar(text_frame)
scrollbar.pack(side=LEFT, fill=Y)

scrollbar.config(command=textfield.yview)
textfield.config(yscrollcommand=scrollbar.set)

text_frame.pack()

entry = Entry(root)
entry.pack(side=LEFT, padx=(0,5))

button = Button(root, text="Senden")
button.pack(side=LEFT)

root.mainloop()

