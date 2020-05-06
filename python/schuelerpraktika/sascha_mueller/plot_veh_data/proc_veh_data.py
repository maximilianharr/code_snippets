#-*- coding: utf-8 -*-
def plot( sensor_id, x_id, y_id ):
  #Für die Parser Funktion
  import xml.etree.ElementTree as ET
  import matplotlib.pyplot as plt

  # Init tree for reading XML
  tree = ET.parse('20160908_rtk_05_insignia.xml')
  root = tree.getroot()

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
  fig, ax = plt.subplots()
  # ax.set_xlim(x[0], x[-1])
  # ax.set_ylim(min(y), max(y))
  plt.plot(x,y,'gx')
  plt.grid()
  plt.show()
  return
