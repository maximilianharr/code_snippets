/**
 *  @file online_calibration_handler.hpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 06.07.2016
 *
 *  @brief Calibrates wheel diameter, time delay, IMU scale, ...
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
#ifndef NODENAMEHANDLER_HPP /* Header File Guards. Prevent bad effect from double include. */
#define NODENAMEHANDLER_HPP

// SYSTEM INCLUDES
#include <vector>

// PROJECT INCLUDES
#include <util_opel_ros/handle_ros_msg.h>
#include <util_opel_ros/ros_param.h>

// LOCAL INCLUDES

// FORWARD REFERENCES

//// CLASS DEFINITION //////////////////////////////////////////////////////////////////////////////
class NodeNameHandler : public HandleRosMsg
{
  public:     /* Acessible from anywhere, where object is visible */


  /* ROS Varialbes */
  ros::NodeHandle ros_nh;
  RosParam ros_param; /* ros parameter class */

// LIFECYCLE
  /** @brief Default constructor.
   *  @param
   *  @return
   */
  NodeNameHandler();

  /** @brief Constructor.
   *  @param
   *  @return
   */
  NodeNameHandler(ros::NodeHandle parent_nh);

  /** @brief Destructor.
   *  @param
   *  @return
   */
  ~NodeNameHandler(void);

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

  /** @brief Callback function for Wheeltick signals (driven axle). Repeats with 100 Hz.
   *  @param
   *  @return
   */
  void wssd_callback(const opel_msgs::OpelCAN_WSSD::ConstPtr&);

  /** @brief Callback function for Wheeltick signals (nondriven axle). Repeats with 100 Hz.
   *  @param
   *  @retur
   */
  void wssnd_callback(const opel_msgs::OpelCAN_WSSND::ConstPtr&);

// ACCESS (attribute accessors)

// INQUIRY (Is*-methods)

  protected:  /* Accessible from members and members of derived classes as well as friends */

  private:    /* Only accessible members or friends */

};

#endif // NODENAMEHANDLER_HPP
