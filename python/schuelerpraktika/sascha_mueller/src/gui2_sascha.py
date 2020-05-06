#-*- coding: utf-8 -*-
import Tkinter, Tkconstants, tkFileDialog
from Tkinter import *
import tkMessageBox
import proc_veh_data as pvd 
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
import matplotlib
matplotlib.use('TkAgg')
import search_file 


#def sensor_imu():
 # sensor_id = 0 
  #tkMessageBox.showinfo(" Sensor_id :", "sensor_id = 0") 
########################################################################

# Define global variables for user input
sensor_id = 0;
x_id = 0;
y_id = 1;
hold_on = 0;
my_colors = ["r","g","b"]

# Read XML file

xml_name = '20160908_rtk_05_insignia.xml'
tree = ET.parse(xml_name)
root = tree.getroot()

# plot in window
top = Tkinter.Tk()
f = Figure(figsize=(5, 4), dpi=100)
canvas = FigureCanvasTkAgg(f, master=top)
toolbar = NavigationToolbar2TkAgg(canvas, top)


########################################################################

def save_hold_on():
  global hold_on
  hold_on = abs(hold_on -1)

def save_x_id(x_val):
  global x_id
  x_id = x_val

def save_y_id(y_val):
  global y_id
  y_id = y_val

def save_sensor_id(tag_id):
  
  # Store sensor id
  global sensor_id
  sensor_id = tag_id
  print sensor_id
  
  # Count sensor outputs
  sensor_string = root[sensor_id].text
  sensor_attr = sensor_string.split(';')
  sensor_size = sensor_attr[0].split(',')
  
  # For each sensor output add a command
  x_achsemenu.delete(0,END)
  y_achsemenu.delete(0,END)
  for iter_x in range( len(sensor_size) ):
    x_achsemenu.add_command(label=iter_x, command=lambda x_val=iter_x : save_x_id(x_val) )
    y_achsemenu.add_command(label=iter_x, command=lambda y_val=iter_x : save_y_id(y_val) )

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
  for i in range(len(imu_attr)-1):
    if x_id == 0:  
      x[i] = float(imu_data[i][x_id])-float(imu_data[0][x_id])
    else:
      x[i] = float(imu_data[i][x_id])

    y[i] = float(imu_data[i][y_id])

  # Plot data
  if hold_on==0:
    print "clear"
    a.clear()
  color_id = min( 2, len(a.lines) )
  a.plot(x,y,'-', color=my_colors[color_id])

  # a tk.DrawingArea  
  canvas.show()
  canvas.get_tk_widget().pack()

  toolbar.update()
  canvas._tkcanvas.pack()

########################################################################


top.title("Plot vehicle data")
top.geometry("800x450")
menu = Menu(top)
top.config(menu=menu)
filemenu = Menu(menu)



# Make Menu
menu.add_cascade(label="SENSOR", menu=filemenu)
for i in range( len(root.findall('*'))-1 ):
  tag_name = root[i].tag
  filemenu.add_command(label=tag_name, command=lambda tag_id=i : save_sensor_id(tag_id) )

x_achsemenu = Menu(menu)
menu.add_cascade(label="X-Achse", menu=x_achsemenu) 

y_achsemenu = Menu(menu)
menu.add_cascade(label="Y-Achse", menu=y_achsemenu)

menu.add_radiobutton(label="Plot", command=print_plot)
menu.add_checkbutton(label="Hold on", command=save_hold_on)

def datei():
  class TkFileDialogExample(Tkinter.Frame):
    
    def __init__(self, root):
      Tkinter.Frame.__init__(self, root)
      root.title("Search maschine")
      root.geometry('300x225')
      # options for buttons
      button_opt = {'fill': Tkconstants.BOTH, 'padx': 5, 'pady': 5}
      # define buttons
      Tkinter.Button(self, text='askopenfile', command=self.askopenfile).pack(**button_opt)
      Tkinter.Button(self, text='askopenfilename', command=self.askopenfilename).pack(**button_opt)
      Tkinter.Button(self, text='asksaveasfile', command=self.asksaveasfile).pack(**button_opt)
      Tkinter.Button(self, text='asksaveasfilename', command=self.asksaveasfilename).pack(**button_opt)
      Tkinter.Button(self, text='askdirectory', command=self.askdirectory).pack(**button_opt)
      # define options for opening or saving a file
      self.file_opt = options = {}
      options['defaultextension'] = '.txt'
      options['filetypes'] = [('all files', '.*'), ('text files', '.txt')]
      options['initialdir'] = 'C:\\'
      options['initialfile'] = 'myfile.txt'
      options['parent'] = root
      options['title'] = 'This is a title'
      # This is only available on the Macintosh, and only when Navigation Services are installed.
      #options['message'] = 'message'
    
      # if you use the multiple file version of the module functions this option is set automatically.
      #options['multiple'] = 1
    
      # defining options for opening a directory
      self.dir_opt = options = {}
      options['initialdir'] = 'C:\\'
      options['mustexist'] = False
      options['parent'] = root
      options['title'] = 'This is a title'

    def askopenfile(self):
       """Returns an opened file in read mode."""
       return tkFileDialog.askopenfile(mode='r', **self.file_opt)

    def askopenfilename(self):

      """Returns an opened file in read mode.
      This time the dialog just returns a filename and the file is opened by your own code.
      """

      # get filename
      filename = tkFileDialog.askopenfilename(**self.file_opt)
      # open file on your own
      if filename:
        return open(filename, 'r')

    def asksaveasfile(self):
  
      """Returns an opened file in write mode."""
   
      return tkFileDialog.asksaveasfile(mode='w', **self.file_opt)
   
    def asksaveasfilename(self):
   
      """Returns an opened file in write mode.
      This time the dialog just returns a filename and the file is opened by your own code.
      """
   
      # get filename
      filename = tkFileDialog.asksaveasfilename(**self.file_opt)
      print filename
   
    def askdirectory(self):
   
      """Returns a selected directoryname."""
   
      return tkFileDialog.askdirectory(**self.dir_opt)

  if __name__=='__main__':
    root = Tkinter.Tk()
    TkFileDialogExample(root).pack()
    root.mainloop()
searchmenu = Menu(menu)
menu.add_cascade(label="Search File", menu=searchmenu)
searchmenu.add_command(label="Open search maschine", command = datei)
top.mainloop()





