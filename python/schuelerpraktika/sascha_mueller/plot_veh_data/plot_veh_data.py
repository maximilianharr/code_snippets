#-*- coding: utf-8 -*-
def port():
#Für die Parser Funktion
  import xml.etree.ElementTree as ET
  import matplotlib.pyplot as plt

# Init tree for reading XML
  tree = ET.parse('20160908_rtk_05_insignia.xml')
  root = tree.getroot()

# Set parameters
  sensor_id = int(input("Bitte 0 Eintippen"))
  meas_id = int(input("O = time stamp, 1 = rateYawAngle, 2 = accelerationLateral,\nBitte eine Zahl wählen."))

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
    x[i] = float(imu_data[i][0])-float(imu_data[0][0])
    y[i] = float(imu_data[i][meas_id])

# Plot data
  fig, ax = plt.subplots()
  ax.set_xlim(x[0], x[-1])
  ax.set_ylim(min(y), max(y))
  plt.plot(x,y,'gx')
  plt.grid()
  plt.show()
  return
