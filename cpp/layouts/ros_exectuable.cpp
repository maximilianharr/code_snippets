/**
 *  @file gnss_provider.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 04.01.2016
 *
 *  @brief Reads GNSS data from GNSS-receiver via chardev.
 *
 *
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
 *
 *
 *  @bug
 *
 *
 *  @todo
 *
 *
 */

// PRAGMA

// SYSTEM INCLUDES
#include <iostream>  /* Header that defines the standard input/output stream objects */
#include <cstdlib>  /* Header that defines several general purpose functions */
#include <string>

// PROJECT INCLUDES
#include "ros/ros.h" /* Convenience include that includes most common pieces of ROS system */

// LOCAL INCLUDES
#include "../include/general_fun.h" /* Includes general functions */

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

/** @brief Standard command line parameter processing.
 *  @param Pass command line parameters
 *  @return 0 if -h flag is set
 */
int cmd_check(int, char*[]);

// GLOBAL VARIABLES

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  /* Check command line options. Stop execution if -h is set. */
  if(!cmd_check(argc,argv)) return 0;

  /* Declare variables */
  std::string node_name, node_sen_ns;

  /* Create ROS-node-handle and store node name */
  ros::init(argc, argv, "gnss_provider");
  node_name = ros::this_node::getName();
  ros::NodeHandle ros_nh;

  /* Read ROS-parameter */
  ros_nh.param<std::string>("/sensor_namespace", node_sen_ns, "/sensor/can");
  ros_nh.param<std::string>("/opel_namespace", node_sen_ns, "/opel");

  /* Enters loop, pumping callbacks. Exits when ros::ok() returns false */
  ros::spin();

  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////
int cmd_check(int argc, char* argv[])
{
  int option;
  /* third argument of getopt specifies valid options and whether they need input(:) */
  while((option = getopt(argc,argv,"hp:"))>=0)
  {
    switch (option)
    {
      case 'h': std::cout
                << "Usage: <filename> [options] \n\n"
                << "<desription> \n\n"
                << "Options: \n"
                << " -h                    show this help message and exit \n"
                << " -p <PARAMETER>        <description> \n"
                << " \n";
                return 0; /* do not execute main with this option */
      case 'p': std::cout << "-p = " << optarg << "\n"; /* optarg is option argument */
                break;
    }
  }
  return 1;
}


