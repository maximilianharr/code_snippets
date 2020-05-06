#!/usr/bin/env python

#  @file io_handler.py
#  @author Maximilian Harr <maximilian.harr@gmail.com>
#  @date 05.02.2017
#
#  @brief Listens to ROS messages and sends IO commands using RPi.GPIO.
#         Note that multiple Nodes may not accees the same port without freeing the port before.
#
#
#
#         Coding Standard:
#         wiki.ros.org/CppStyleGuide
#         https://google.github.io/styleguide/cppguide.html
#
#
#  @bug
#
#
#  @todo 
#
#

# Import ROS libraries and messages
import rospy
from std_msgs.msg import String
from pi_msgs.msg import PiLED
from pi_msgs.msg import PiMotor
from pi_msgs.msg import PiPlug

# Other imports
import os
import RPi.GPIO as GPIO

# Alias for Ports
GPIO_LED = 21
# @todo Motor

# Set ports
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_LED, GPIO.OUT)

def led_callback(data):
    if(data.state):
        GPIO.output(GPIO_LED, True)
    else:
        GPIO.output(GPIO_LED, False)

def motor_callback(data):
    # Do something
    a = 1

def plug_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Received plug data: ' + data)
    os.system('sudo /home/ubuntu/raspberry_ws/src/action_pi/include/raspberry_remote/send 10010 ' + data.int + ' ' + data.state)

# Main routine
def io_handler():
    # Init Node and Subscriber
    rospy.init_node('io_handler', anonymous=True)
    rospy.Subscriber('/io_command/led', PiMotor, led_callback)
    rospy.Subscriber('/io_command/motor', PiMotor, motor_callback)
    rospy.Subscriber('/io_command/plug', PiMotor, plug_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    # Cleanup
    GPIO.cleanup()

if __name__ == '__main__':
    io_handler()
