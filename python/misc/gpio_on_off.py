
# Ein-/Ausschalten von GPIO

import os
import time
import RPi.GPIO as GPIO
 
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)
 
for i in range(2):

  GPIO.output(21, True)
  print 'on'
  time.sleep(1)

  GPIO.output(21,False)
  print 'off'
  time.sleep(1)

os.system('sudo ./raspberry-remote/send 10010 1 0')
time.sleep(1)
os.system('sudo ./raspberry-remote/send 10010 1 1')

GPIO.cleanup()

