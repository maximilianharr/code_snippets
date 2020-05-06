#!/usr/bin/env python

#  @file bar_chart.py
#  @author Maximilian Harr <maximilian.harr@gmail.com>
#  @date 22.11.2017
#
#  @brief Animated bar chart
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
import numpy as np

# Data to plot
labels = 'Python', 'C++', 'Ruby', 'Java'
values = [2, 2, 2, 2]
explode = (0.1, 0, 0, 0)  # explode 1st slice
width = 0.5 # the width of the bars

# Plot 
fig, ax = plt.subplots()
ax.set_xlim([0,100])
ax.set_ylim([0,100])
ind = np.arange(len(values))  # the x locations for the groups

while 1:
  values[2] += 1
  y_pos = np.arange(len(labels))
  ax.barh(ind, values, color='blue')
  ax.set_yticks(ind+width/2)
  ax.set_yticklabels(labels, minor=False)
  ax.set_xlabel('Percent %')
  ax.set_title('CPU Usage')
  for i, v in enumerate(values):
    ax.text(v, i + .25, str(v)+' %', color='blue', fontweight='bold')
  plt.pause(0.5)
  plt.draw()
  ax.clear()
