// File:           myCanReader.cpp
// Creation Date:  09.11.2015
// Author:         Maximilian Harr <maximilian.harr@daimler.com>
// Miscellaneous:  wiki.ros.org/CppStyleGuide
// 
// Description:    
//  Example/Template for ROS subscriber.
//                 
//////////////////////////////////////////////////////////////////////

// HEADERS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

// PRAGMA

// FUNCTION DEFINITONS

using namespace std;
int main(int argc, char **argv)
{
  // Initialize ROS, assign name to node
  ros::init(argc, argv, "myCanReader");
  // Create node handle for process' node
  ros::NodeHandle n;

  std::cout << "Hello";
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
    << "Usage: rosrun myCanReader\n \n"
    << "Commands: \n"
    << "Options: \n \n";
    return true;
  }
  return false;
}

