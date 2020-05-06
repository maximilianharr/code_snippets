/**
 *  @file odo_ros_handler.hpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 01.04.2016
 *
 *  @brief rosmsg_grab distributes ros-msgs to this class, which then distributes messages to
 *         according functions (depending on the package).
 *         Shared functionality implemented in opel_util_ros pkg -> handle_ros_msg.h
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
#ifndef ODOROSHANDLER_HPP /* Header File Guards. Prevent bad effect from double include. */
#define ODOROSHANDLER_HPP /* Add Namespace if used. NAMSPACE_SENSORFILTER_H */

// SYSTEM INCLUDES
#include <vector>

// PROJECT INCLUDES
#include <geographic_msgs/GeoPoint.h>
#include <util_opel_ros/handle_ros_msg.h>
#include <util_opel_ros/ros_param.h>
#include <sensor_fusion_ros/wss_odo.h>

// LOCAL INCLUDES

// FORWARD REFERENCES

//// CLASS DEFINITION //////////////////////////////////////////////////////////////////////////////
class OdoRosHandler : public HandleRosMsg
{
  public:     /* Acessible from anywhere, where object is visible */

  /* Bools */
  bool b_init_complete;

  /* Wheel Speed Odometer */
  RosParam ros_param;
  WssOdo wss_odo;

  /* ROS publisher */
  ros::NodeHandle ros_nh;
  ros::Publisher pub;
  geographic_msgs::GeoPoint wssd_geopoint;


// LIFECYCLE
  /** @brief Default constructor.
   *  @param
   *  @return
   */
  OdoRosHandler();

  /** @brief Constructor.
   *  @param
   *  @return
   */
  OdoRosHandler(ros::NodeHandle parent_nh);

  /** @brief Destructor.
   *  @param
   *  @return
   */
  ~OdoRosHandler(void);

// OPERATORS

// OPERATIONS (all non-access/inquiry methods)
  /** @brief Callback function for GPS signals
   *  @param
   *  @return
   */
  void gps_callback(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr&);

  /** @brief Callback function for IMU signals
   *  @param
   *  @return
   */
  void imu_callback(const opel_msgs::OpelCAN_IMU::ConstPtr&);

  /** @brief Callback function for IMU signals
   *  @param
   *  @return
   */
  void imu2_callback(const opel_msgs::OpelCAN_IMU2::ConstPtr&);

  /** @brief Callback function for Steering signals
   *  @param
   *  @return
   */
  void steer_callback(const opel_msgs::OpelCAN_Steer::ConstPtr&);

  /** @brief Callback function for Steering signals
   *  @param
   *  @return
   */
  void trigger_callback(const opel_msgs::OpelCAN_Trigger::ConstPtr&);

  /** @brief Callback function for Wheeltick signals (driven axle). Repeats with 100 Hz.
   *  @param
   *  @return
   */
  void wssd_callback(const opel_msgs::OpelCAN_WSSD::ConstPtr&);

  /** @brief Callback function for Wheeltick signals (nondriven axle). Repeats with 100 Hz.
   *  @param
   *  @return
   */
  void wssnd_callback(const opel_msgs::OpelCAN_WSSND::ConstPtr&);

  /** @brief Callback function for ADMA signals. Repeats with 100 Hz.
   *  @param
   *  @return
   */
  void adma_callback(const mrt_sensor_msgs_ros::ADMA::ConstPtr&);

  /** @brief Checks if initialization is complete
   *  @param
   *  @return
   */
  void init_complete();

// ACCESS (attribute accessors)

// INQUIRY (Is*-methods)

  protected:  /* Accessible from members and members of derived classes as well as friends */

  private:    /* Only accessible members or friends */

};

#endif // ODOROSHANDLER_HPP
