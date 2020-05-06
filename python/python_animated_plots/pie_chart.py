#!/usr/bin/env python

#  @file pie_chart.py
#  @author Maximilian Harr <maximilian.harr@gmail.com>
#  @date 22.11.2017
#
#  @brief Animated py chart
#
#
#
#  @bug
#
#
#  @todo 
#
#


import matplotlib.pyplot as plt
import time

# Data to plot
labels = 'Python', 'C++', 'Ruby', 'Java'
values = [2, 2, 2, 2]
explode = (0.1, 0, 0, 0)  # explode 1st slice

# Plot 
plt.ion()
fig, ax = plt.subplots()
plt.axis('equal')
while 1:
  values[2] += 1
  ax.pie(values, labels=labels, shadow=True, autopct='%f %%', startangle=140)
  # ax.pie(values, explode=explode, labels=labels, autopct='%1.1f%%', shadow=True, startangle=140)
  
  plt.pause(0.5)
  plt.draw()
  ax.clear()

