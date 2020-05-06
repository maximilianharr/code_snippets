#!/usr/bin/env python

#  @file fig2tikz.py
#  @author Maximilian Harr <maximilian.harr@gmail.com>
#  @date 20.12.2017
#
#  @brief Opens a figure and saves as tikz
#         pip install -U matplotlib2tikz
#         Before running: source ~/.virtualenvs/tikz/bin/activate
#
#  @bug
#
#
#  @todo 
#
#

import pylab
import matplotlib.pyplot as plt
import numpy as np
import imp
import sys
import string
import argparse

def convert_fig_to_tikz(xml_file):
  # Get location from xml_file-name
  xml_location = ''
  if xml_file == None:
    xml_location = './'
  else:
    if len(xml_file.split('/')[0:-1]) == 0:
      xml_location = './'
    else:
      xml_location = '/'.join(xml_file.split('/')[0:-1])+'/'

  # Define Colors for output
  class bcolors:
      OKGREEN = '\033[92m'
      WARNING = '\033[93m'
      FAIL = '\033[91m'
      ENDC = '\033[0m'
  print bcolors.OKGREEN + "Start converting figures to tikz ..." + bcolors.ENDC

  # Try to import matplotlib2tikz
  try:
      imp.find_module('matplotlib2tikz')
      print bcolors.OKGREEN + "Found matplotlib2tikz. Converting to tikz ..." + bcolors.ENDC
      from matplotlib2tikz import save as tikz_save
  except ImportError:
    print bcolors.FAIL + "Cannot generate tikz. Unable to find matplotlib2tikz. Install using: " + bcolors.ENDC
    print bcolors.FAIL + "pip install -U matplotlib2tikz" + bcolors.ENDC
    print bcolors.FAIL + "Note: Ideally use new virtual python environment." + bcolors.ENDC
    sys.exit()

  # Iterate through figures and store as png and tex
  # https://matplotlib.org/api/_as_gen/matplotlib.figure.Figure.html#matplotlib.figure.Figure
  for i, manager in enumerate(plt._pylab_helpers.Gcf.get_all_fig_managers()):
    old_title = manager.window.wm_title()
    new_title = string.replace(old_title, ' ', '_')
    print bcolors.OKGREEN + ("Processing figure #%i ..." % i) + bcolors.ENDC
    manager.canvas.figure.savefig('%s.png' % new_title )
    filename = xml_location + '%s.tex' % new_title
    tikz_save( filename, manager.canvas.figure )
    print bcolors.OKGREEN + ("Saved figure #%i :" % i + filename) + bcolors.ENDC

  print bcolors.OKGREEN + "Finished." + bcolors.ENDC

def make_plots():
  # Create sine and cosine plot
  t = np.arange(0.0, 2.0, 0.1)
  s = np.sin(2*np.pi*t)
  c = np.cos(2*np.pi*t)

  plt.figure()
  fig = pylab.gcf()
  title = 'Sinus 1'
  plt.title(title)
  fig.canvas.set_window_title(title)
  plt.plot(t, s, 'o-', lw=4.1)

  plt.figure()
  fig = pylab.gcf()
  title = 'Cosinus 1'
  plt.title(title)
  fig.canvas.set_window_title(title)
  plt.plot(t, c, 'o-', lw=4.1)

if __name__ == '__main__':
  print "Hello"
  # https://docs.python.org/2/library/argparse.html
  parser = argparse.ArgumentParser()
  parser.add_argument('--xml_file', type=None,
    help='Location of xml-file created by opel_helper_ros_tool/xml_converter')
  parser.add_argument('--tikz', type=None,
    help='Creates tikz-files if [true]')
  args = parser.parse_args()
  print args.xml_file
  if args.tikz != None:
    if args.tikz.lower() == 'true':
      print "Creating tikz"
      make_plots()
      convert_fig_to_tikz(args.xml_file)
