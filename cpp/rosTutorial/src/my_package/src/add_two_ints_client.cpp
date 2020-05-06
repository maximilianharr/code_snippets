// File:           add_two_ints_client.cpp
// Creation Date:  02.11.2015
// Author:         Maximilian Harr <maximilian.harr@daimler.com>
// Miscellaneous:  wiki.ros.org/CppStyleGuide
//
// Description:
//  Example/Template for ROS client.
//
//////////////////////////////////////////////////////////////////////

// HEADERS
#include "ros/ros.h"
#include "my_package/AddTwoInts.h"
#include <cstdlib>

// PRAGMA

// FUNCTION DEFINITONS

int main(int argc, char **argv)
{
  // Initialize ROS, assign name to node
  ros::init(argc, argv, "add_two_ints_client");

  // Check if correct amount of input is provided
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  // Create node handle for process' node
  ros::NodeHandle n;

  // Create ROS client
  ros::ServiceClient client = n.serviceClient<my_package::AddTwoInts>("add_two_ints");

  // Create Service Class and assign request member values.
  my_package::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  // Call the service
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
  return 0;
}


//////////////////////////////////////////////////////////////////////
// Help tool for command line:
bool help(int argc, char ** argv)
{
  if(argc == 2 && !strcmp(argv[1],"-h"))
  {
    std::cout << argv[0] << " is a function for testing purposes.\n \n"
    << "Usage: rosrun add_two_ints_server\n \n"
    << "Commands: \n"
    << "Options: \n \n";
    return true;
  }
  return false;
}

