// File:           mySubsriber.cpp
// Creation Date:  01.11.2015
// Author:         Maximilian Harr <maximilian.harr@daimler.com>
// Miscellaneous:  wiki.ros.org/CppStyleGuide
// 
// Description:    
//  Example/Template for ROS subscriber.
//                 
//////////////////////////////////////////////////////////////////////

// HEADERS
#include "ros/ros.h"
#include "std_msgs/String.h"

// PRAGMA

// FUNCTION DEFINITONS
bool help(int, char **);
// Callback function that gets called when new message arrives.
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

using namespace std;
int main(int argc, char **argv)
{
  // Initialize ROS, assign name to node
  ros::init(argc, argv, "listener");
  // Create node handle for process' node
  ros::NodeHandle n;
  // Tell ROS which topic you want to subscribe to
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // Enters loop, pumping callbacks. Exits when ros::ok() returns false
  ros::spin();
  return 0;
}


//////////////////////////////////////////////////////////////////////
// Help tool for command line:
bool help(int argc, char ** argv)
{
  if(argc == 2 && !strcmp(argv[1],"-h"))
  {
    std::cout << argv[0] << " is a function for testing purposes.\n \n"
    << "Usage: rosrun mySubscriber\n \n"
    << "Commands: \n"
    << "Options: \n \n";
    return true;
  }
  return false;
}

