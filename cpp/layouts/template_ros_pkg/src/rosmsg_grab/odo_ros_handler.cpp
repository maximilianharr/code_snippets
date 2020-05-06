/**
 *  @file odo_ros_handler.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 01.04.2016
 *
 *  @brief	See odo_ros_handler.h
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
#include "odo_ros_handler.h"

// FORWARD REFERENCES

//// METHOD DEFINITION /////////////////////////////////////////////////////////////////////////////

// LIFECYCLE
OdoRosHandler::OdoRosHandler(){}

OdoRosHandler::OdoRosHandler(ros::NodeHandle parent_nh)
{
  wss_odo.setParam(ros_param.param);
  ROS_INFO_STREAM(ros_param.vehicle.l_b);
  wss_odo.setPose(0,0,0);
  b_init_complete = false;

  ros_nh = parent_nh;
  pub = ros_nh.advertise<geographic_msgs::GeoPoint>(ros_param.name.opel + "/wss_odo",1);
}

OdoRosHandler::~OdoRosHandler(void)
{

}

// OPERATORS

// OPERATIONS (all non-access/inquiry methods)
void OdoRosHandler::gps_callback(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg)
{
  /* Convert message and update message cache */
  gnss_rtg(msg, &gnss_data);
  update_cache(&gnss_cache, gnss_data);


}

void OdoRosHandler::imu_callback(const opel_msgs::OpelCAN_IMU::ConstPtr& msg)
{
  /* Convert message and update message cache */
  imu_rtg(msg, &imu_data);
  update_cache(&imu_cache, imu_data);

}

void OdoRosHandler::imu2_callback(const opel_msgs::OpelCAN_IMU2::ConstPtr& msg)
{
  /* Convert message and update message cache */
  imu2_rtg(msg, &imu2_data);
  update_cache(&imu2_cache, imu2_data);

}

void OdoRosHandler::steer_callback(const opel_msgs::OpelCAN_Steer::ConstPtr& msg)
{
  /* Convert message and update message cache */
  steer_rtg(msg, &steer_data);
  update_cache(&steer_cache, steer_data);

}

void OdoRosHandler::trigger_callback(const opel_msgs::OpelCAN_Trigger::ConstPtr&)
{

}

void OdoRosHandler::wssd_callback(const opel_msgs::OpelCAN_WSSD::ConstPtr& msg)
{
  /* Convert message and update message cache */
  wssd_rtg(msg, &wssd_data);
  update_cache(&wssd_cache, wssd_data);

  if(b_init_complete)
  {
    /* Run WssOdo-Node */
    wss_odo.processWSSD(wssd_cache);

    /* @todo convert utm to wgs84 with geodesy */
    wssd_geopoint.latitude = wss_odo.x_k1;
    wssd_geopoint.longitude = wss_odo.y_k1;
    wssd_geopoint.altitude = 0;
    pub.publish(wssd_geopoint);
  }
  else
  {
    init_complete();
  }

}

void OdoRosHandler::wssnd_callback(const opel_msgs::OpelCAN_WSSND::ConstPtr& msg)
{
  /* Convert message and update message cache */
  wssnd_rtg(msg, &wssnd_data);
  update_cache(&wssnd_cache, wssnd_data);

}

void OdoRosHandler::adma_callback(const mrt_sensor_msgs_ros::ADMA::ConstPtr& msg)
{
  /* Convert message and update message cache */
  adma_rtg(msg, &adma_data);
  update_cache(&adma_cache, adma_data);
}

void OdoRosHandler::init_complete()
{
  if( (wssd_cache.size()>1) && (wssnd_cache.size()>1) )
  {
    /* @todo Initialize position and orientation (GPS) and set GeoPose to it */
    b_init_complete = true;
  }
}

// ACCESS (attribute accessors)

// INQUIRY (Is*-methods)


