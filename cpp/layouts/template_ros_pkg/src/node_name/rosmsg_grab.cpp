/**
 *  @file rosmsg_grab.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 06.07.2016
 *
 *  @brief Takes ROS messages and distributes them to handle class
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

// PROJECT INCLUDES

// LOCAL INCLUDES
#include "node_name_handler.h"

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

  /* Create ROS-node-handle and store node name */
  ros::init(argc, argv, "can_odometer");
  ros::NodeHandle ros_nh;
  RosParam ros_param; /* object with global parameters */

  /* Subscribe to sensor data and call member function of sensor-handler */

  NodeNameHandler odo_ros_handler(ros_nh);
  ros::Subscriber sensor_gps = ros_nh.subscribe(ros_param.name.sensor + "/ublox/data"
                                              ,1000,&NodeNameHandler::gps_callback,&odo_ros_handler);
  ros::Subscriber sensor_imu = ros_nh.subscribe(ros_param.name.sensor + "/imu"
                                              ,1000,&NodeNameHandler::imu_callback,&odo_ros_handler);
  ros::Subscriber sensor_imu2 = ros_nh.subscribe(ros_param.name.sensor + "/imu2"
                                              ,1000,&NodeNameHandler::imu2_callback,&odo_ros_handler);
  ros::Subscriber sensor_steer = ros_nh.subscribe(ros_param.name.sensor + "/steer"
                                              ,1000,&NodeNameHandler::steer_callback,&odo_ros_handler);
  ros::Subscriber sensor_wssd = ros_nh.subscribe(ros_param.name.sensor + "/wssd"
                                              ,1000,&NodeNameHandler::wssd_callback,&odo_ros_handler);
  ros::Subscriber sensor_wssnd = ros_nh.subscribe(ros_param.name.sensor + "/wssnd"
                                              ,1000,&NodeNameHandler::wssnd_callback,&odo_ros_handler);

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
                << "Usage: rosmsg_grab\n\n"
                << "Subscribes to Opel-CAN-data and distributes ROS-messages to NodeNameHandler-class. \n\n"
                << "Options: \n"
                << " -h                    show this help message and exit \n"
                << " \n";
                return 0; /* do not execute main with this option */
    }
  }
  return 1;
}

