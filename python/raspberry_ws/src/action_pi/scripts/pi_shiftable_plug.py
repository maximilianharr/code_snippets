#!/usr/bin/env python

#  @file pi_shiftable_plug.py
#  @author Maximilian Harr <maximilian.harr@gmail.com>
#  @date 05.02.2017
#
#  @brief Turns on/off shiftable plugs depending on IMU acceleration of phone
#
#
#
#         Coding Standard:
#         wiki.ros.org/CppStyleGuide
#         https://google.github.io/styleguide/cppguide.html
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
from sensor_msgs.msg import Imu
from pi_msgs.msg import PiLED

# Other imports


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s ', data.linear_acceleration.z)

def pi_shiftable_plug():
    # Init Node, Publisher and Subscriber
    rospy.init_node('pi_shiftable_plug', anonymous=True)
    rospy.Subscriber('/phone1/android/imu', Imu, callback)
    pub_plug = rospy.Publisher('/io_command/plug', PiLED, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pi_shiftable_plug()
