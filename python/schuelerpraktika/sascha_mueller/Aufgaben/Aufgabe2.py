#-*- coding: utf-8 -*-
#Sinuskurve im Graphen
# @todo Beschriftung der Achsen
# @todo Legende
# @todo Sinus (gruen) und Cosinus (rot)

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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
plt.plot(x, y, 'gx')

plt.plot(x, y, 'gx', x, f(x, 2, 3), 'r-')

plt.show()


