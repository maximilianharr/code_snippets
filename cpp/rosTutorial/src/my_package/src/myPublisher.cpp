// File:           myPublisher.cpp
// Creation Date:  01.11.2015
// Author:         Maximilian Harr <maximilian.harr@daimler.com>
// Miscellaneous:  wiki.ros.org/CppStyleGuide
// 
// Description:    
//  Example/Template for communication with messages/services
//                 
//////////////////////////////////////////////////////////////////////

// HEADERS
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>

// PRAGMA
#define debug

// FUNCTION DEFINITONS
bool help(int, char **);

int main(int argc, char **argv)
{
  // Initialize ROS and specify ROS internal node name
  ros::init(argc, argv, "publisher");
  // Create node handle to the process' nodes. 
  // (Acces point for communication with ROS system)
  ros::NodeHandle n; 
  
  // Tell ROS you want to publish on chatter. Master-Node informs subscribers 
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000);
  ros::Rate loop_rate(10);
  
  int count=0;

  while(ros::ok())
  {
    // Generate message
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world" << count;
    msg.data = ss.str();
    ++count;    

    // show on console, replaces printf/cout
    ROS_INFO("%s", msg.data.c_str());
    
    // publish message
    chatter_pub.publish(msg);

    // spinOnce: Necessary, if there are subscriptions to this topic
    // sleep: sleep remaint amount of time to hit 10 Hz rate
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

//////////////////////////////////////////////////////////////////////
// Help tool for command line:
bool help(int argc, char ** argv)
{
  if(argc == 2 && !strcmp(argv[1],"-h"))
  {
    std::cout << argv[0] << " is a function for testing messages and services.\n \n"
    << "Usage: rosrun myPublisher\n \n"
    << "Commands: \n"
    << "Options: \n \n";
    return true;
  }
  return false;
}

