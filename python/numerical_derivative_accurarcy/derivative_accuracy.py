# -*- coding: utf-8 -*-
# Source virtual environment for tikz:
# source ~/.virtualenvs/tikz/bin/activate

import numpy as np
import matplotlib.pyplot as plt
from matplotlib2tikz import save as tikz_save


h = np.logspace(-20, 3, num=300)
x_val = [0.01, 0.1, 1.0, 10.0]
#x_val = [0.01]

for i,x in enumerate(x_val):
  y = x*x*x * np.ones( len(h) )
  y_d = 3*x*x * np.ones( len(h) )

  y_h = (x+h)*(x+h)*(x+h) * np.ones( len(h) )
  y_mh = (x-h)*(x-h)*(x-h) * np.ones( len(h) )
  y_d_est = (y_h - y)/h
  y_d_cest = (y_h - y_mh)/(2*h)

  #plt.semilogx(h, abs(y_d - y_d_est))
  plt.loglog(h, abs(y_d - y_d_est), color = [ float(i+2)/(len(x_val)+1), 0.0, 0.0 ])
  plt.loglog(h, abs(y_d - y_d_cest), color = [ 0.0, 0.0, float(i+2)/(len(x_val)+1) ])
  print float(i+2)/(len(x_val)+1)
  plt.xlabel('Stepszie h')
  plt.ylabel('Numerical error')

tikz_save('Numerical_Error_Differentiation.tex')
plt.show()
