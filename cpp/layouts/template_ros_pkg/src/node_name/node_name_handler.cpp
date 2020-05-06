/**
 *  @file node_name_handler.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 06.07.2016
 *
 *  @brief	See node_name_handler.h
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
 */

// PRAGMA

// SYSTEM INCLUDES

// PROJECT INCLUDES

// LOCAL INCLUDES
#include "node_namehandler.h"

// FORWARD REFERENCES

//// METHOD DEFINITION /////////////////////////////////////////////////////////////////////////////

// LIFECYCLE
NodeNameHandler::NodeNameHandler(){}

NodeNameHandler::NodeNameHandler(ros::NodeHandle parent_nh)
{
  /* ROS */
  ros_nh = parent_nh;

}

NodeNameHandler::~NodeNameHandler(void)
{

}

// OPERATORS

// OPERATIONS (all non-access/inquiry methods)
void NodeNameHandler::gps_callback(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg)
{
  /* Convert message and update message cache */
  gnss_rtg(msg, &gnss_data);
  update_cache(&gnss_cache, gnss_data);

}

void NodeNameHandler::imu_callback(const opel_msgs::OpelCAN_IMU::ConstPtr& msg)
{
  /* Convert message and update message cache */
  imu_rtg(msg, &imu_data);
  update_cache(&imu_cache, imu_data);

}

void NodeNameHandler::imu2_callback(const opel_msgs::OpelCAN_IMU2::ConstPtr& msg)
{
  /* Convert message and update message cache */
  imu2_rtg(msg, &imu2_data);
  update_cache(&imu2_cache, imu2_data);

}

void NodeNameHandler::steer_callback(const opel_msgs::OpelCAN_Steer::ConstPtr& msg)
{
  /* Convert message and update message cache */
  steer_rtg(msg, &steer_data);
  update_cache(&steer_cache, steer_data);

}

void NodeNameHandler::wssd_callback(const opel_msgs::OpelCAN_WSSD::ConstPtr& msg)
{
  /* Convert message and update message cache */
  wssd_rtg(msg, &wssd_data);
  update_cache(&wssd_cache, wssd_data);

}

void NodeNameHandler::wssnd_callback(const opel_msgs::OpelCAN_WSSND::ConstPtr& msg)
{
  /* Convert message and update message cache */
  wssnd_rtg(msg, &wssnd_data);
  update_cache(&wssnd_cache, wssnd_data);

}

// ACCESS (attribute accessors)

// INQUIRY (Is*-methods)


