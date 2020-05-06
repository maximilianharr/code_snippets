/**
 *  @file eigen_quaternion.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 17.06.2017
 *
 *  @brief Eigen quaternion computations
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
#include <iostream>
#include <cstdlib>
#include <math.h>

#include <Eigen/Core>

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64MultiArray.h>

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

/** @brief 
 *  @param 
 *  @return 
 */


// GLOBAL VARIABLES

using namespace Eigen;
using namespace std;

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  
  /* Uncertrainty of position and estimation */
  Eigen::MatrixXd mat_position    = 1*Eigen::MatrixXd::Identity(3,3);
  Eigen::MatrixXd mat_orientation = 2*Eigen::MatrixXd::Identity(3,3);

  /* Fill uncertainty of pose (position and orientation) */
  Eigen::MatrixXd mat_pose = Eigen::MatrixXd::Zero(6,6);
  mat_pose.block(0,0,3,3) =  mat_position;
  mat_pose.block(3,3,3,3) =  mat_orientation;

  std::cout << "mat_pose:\n" << mat_pose << std::endl;

  /* @todo delete. Mapping of Eigen Matrix in std::vector and inserting in MutliArray */
  std::vector<double> covmat_std(mat_pose.data(), mat_pose.data() + mat_pose.rows() * mat_pose.cols() );
  std_msgs::Float64MultiArray array_msg;
  array_msg.data.insert( array_msg.data.end(), covmat_std.begin(), covmat_std.end() );
  std::cout << " array_msg \n " << array_msg << std::endl;

  /* Map Eigen::Matrix in PoseWithCovarianceStamped */
  geometry_msgs::PoseWithCovarianceStamped pwcs;
  pwcs.pose.covariance[0] = 0.0;
  Eigen::VectorXd::Map( &pwcs.pose.covariance[0], mat_pose.size() ) = mat_pose;

  return 0;
  
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////
