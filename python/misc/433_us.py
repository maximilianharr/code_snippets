
import os
import RPi.GPIO as GPIO
import time

TRIG = 23
ECHO = 24

print "Distance Measurement In Progress"

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(21, GPIO.OUT)

GPIO.output(TRIG, False)
print "Waiting For Sensor To Settle"
time.sleep(2)

distance = 100

while distance > 10:  
  
  GPIO.output(TRIG, True)
  time.sleep(0.00001)
  GPIO.output(TRIG, False)
  
  GPIO.output(21,True)
  
  

  while GPIO.input(ECHO)==0:
    pulse_start = time.time()

  while GPIO.input(ECHO)==1:
    pulse_end = time.time()       

  pulse_duration = pulse_end - pulse_start

  distance = pulse_duration * 17150
  distance = round(distance, 2)
  print "Distance:",distance,"cm"
  
  GPIO.output(21,False)

  time.sleep(0.2)
  
  if distance < 30:
    os.system('sudo ./raspberry-remote/send 10010 1 1')    
    for i in range(10):
      GPIO.output(21,True); time.sleep(0.1)
      GPIO.output(21,False); time.sleep(0.9)
    os.system('sudo ./raspberry-remote/send 10010 1 0')


GPIO.cleanup()


