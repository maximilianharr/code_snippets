// File:           add_two_ints_server.cpp
// Creation Date:  02.11.2015
// Author:         Maximilian Harr <maximilian.harr@daimler.com>
// Miscellaneous:  wiki.ros.org/CppStyleGuide
//
// Description:
//  Example/Template for ROS server.
//
//////////////////////////////////////////////////////////////////////

// HEADERS
#include "ros/ros.h"
#include "my_package/AddTwoInts.h"

// PRAGMA

// FUNCTION DEFINITONS
bool add(my_package::AddTwoInts::Request &req,
         my_package::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

using namespace std;
int main(int argc, char **argv)
{
  // Initialize ROS, assign name to node
  ros::init(argc, argv, "add_two_ints_server");
  // Create node handle for process' node
  ros::NodeHandle n;
  //
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
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
    << "Usage: rosrun add_two_ints_server\n \n"
    << "Commands: \n"
    << "Options: \n \n";
    return true;
  }
  return false;
}
