#!/usr/bin/env python

#  @file bar_chart.py
#  @author Maximilian Harr <maximilian.harr@gmail.com>
#  @date 22.11.2017
#
#  @brief Creates a bar chart and converts to tikz
#         Make and source a virtual environment and 
#         pip install -U matplotlib2tikz
#         Before running: source ~/.virtualenvs/tikz/bin/activate
#
#  @bug
#
#
#  @todo 
#
#


import matplotlib.pyplot as plt
import numpy as np
from matplotlib2tikz import save as tikz_save

# Data to plot
labels = 'Python', 'C++', 'Ruby', 'Java'
values = [2, 2, 2, 2]
explode = (0.1, 0, 0, 0)  # explode 1st slice
width = 0.5 # the width of the bars

# Plot 
fig, ax = plt.subplots()
ax.set_xlim([0,100])
ind = np.arange(len(values))  # the x locations for the groups

values[2] += 1
y_pos = np.arange(len(labels))
ax.barh(ind, values, color='blue')
ax.set_yticks(ind)
ax.set_yticklabels(labels, minor=False)
ax.set_xlabel('Percent %')
ax.set_title('CPU Usage')
for i, v in enumerate(values):
  ax.text(v, i, str(v)+' %', color='blue', fontweight='bold')
# plt.show()
tikz_save('barplot.tex')

# import matplotlib.pyplot as plt
# import numpy as np

# plt.style.use('ggplot')

# t = np.arange(0.0, 2.0, 0.1)
# s = np.sin(2*np.pi*t)
# s2 = np.cos(2*np.pi*t)
# plt.plot(t, s, 'o-', lw=4.1)
# plt.plot(t, s2, 'o-', lw=4.1)
# plt.xlabel('time (s)')
# plt.ylabel('Voltage (mV)')
# plt.title('Simple plot $\\frac{\\alpha}{2}$')
# plt.grid(True)

# from matplotlib2tikz import save as tikz_save
# tikz_save('test.tex')