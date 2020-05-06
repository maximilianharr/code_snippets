/**
 *  @file eigen_vector_operations.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 13.07.2017
 *
 *  @brief Eigen vector computations
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
#include <Eigen/Dense>  /* template library for linear algebra http://eigen.tuxfamily.org */
#include <math.h> /* Defines M_PI for pi value */
#include <Eigen/Eigenvalues> 
#include <Eigen/Geometry>

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES
void ProjectVector(Eigen::VectorXd vec_a, Eigen::VectorXd vec_b, 
  Eigen::VectorXd& vec_parallel, Eigen::VectorXd& vec_perpendicular );

void ProjectVectorAbsolute(Eigen::VectorXd vec_a, Eigen::VectorXd vec_b, 
  double& parallel, double& perpendicular );

void ProjectVectorAbsolute(Eigen::Vector2d vec_a, Eigen::Vector2d vec_b, 
  double& parallel, double& perpendicular );

// GLOBAL VARIABLES


//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  
  Eigen::VectorXd vec_a(3), vec_b(3);

  Eigen::VectorXd vec_parallel(3), vec_perpendicular(3);
  double parallel, perpendicular;

  /* Project b on a */
  vec_a << 2, 0, 0;
  vec_b << 1, 1, 0;
  ProjectVector(vec_a, vec_b, vec_parallel, vec_perpendicular);
  vec_b << -1, 1, 0;
  ProjectVector(vec_a, vec_b, vec_parallel, vec_perpendicular);
  vec_b << -1, -1, 0;
  ProjectVector(vec_a, vec_b, vec_parallel, vec_perpendicular);
  vec_b << 1, -1, 0;
  ProjectVector(vec_a, vec_b, vec_parallel, vec_perpendicular);
  vec_b << 1, 0, 0;
  ProjectVector(vec_a, vec_b, vec_parallel, vec_perpendicular);
  vec_b << 0, 0, 1;
  ProjectVector(vec_a, vec_b, vec_parallel, vec_perpendicular);

  std::cout << "---------- 3d vector rotation in x,y ----------" << std::endl;
  Eigen::Vector2d vec_a3, vec_b3;
  vec_a3 << 2, 0;
  vec_b3 << 1, 1;
  ProjectVectorAbsolute(vec_a3, vec_b3, parallel, perpendicular);
  vec_b3 << -1, 1;
  ProjectVectorAbsolute(vec_a3, vec_b3, parallel, perpendicular);
  vec_b3 << -1, -1;
  ProjectVectorAbsolute(vec_a3, vec_b3, parallel, perpendicular);
  vec_b3 << 1, -1;
  ProjectVectorAbsolute(vec_a3, vec_b3, parallel, perpendicular);

  return 0;

}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////

void ProjectVector(Eigen::VectorXd vec_a, Eigen::VectorXd vec_b, 
  Eigen::VectorXd& vec_parallel, Eigen::VectorXd& vec_perpendicular ){
  
  double parallel, perpendicular;
  ProjectVectorAbsolute(vec_a, vec_b, parallel, perpendicular);

  vec_parallel = vec_a;
  vec_parallel.normalize();
  vec_parallel = parallel*vec_parallel;

  vec_perpendicular = vec_b-vec_parallel;
  vec_perpendicular.normalize();
  vec_perpendicular = perpendicular*vec_perpendicular;

  std::cout << "vec_b            : \n" << vec_b << std::endl;
  std::cout << "vec_parallel     : \n" << vec_parallel << std::endl;
  std::cout << "vec_perpendicular: \n" << vec_perpendicular << std::endl;

}

void ProjectVectorAbsolute(Eigen::VectorXd vec_a, Eigen::VectorXd vec_b, 
  double& parallel, double& perpendicular ){
  
  // Warning: there is no check whether perpendicular portion is left/right use Vector2d function

  /* rotation from a to b (Kosinussatz) */
  double phi = acos( vec_a.dot(vec_b)/ (vec_a.norm()*vec_b.norm()) );

  parallel = cos(phi)*vec_b.norm();
  perpendicular = sin(phi)*vec_b.norm();
  std::cout << std::endl << "---------------" << std::endl;
  std::cout << "phi          : " << phi << std::endl;
  std::cout << "parallel     : " << parallel << std::endl;
  std::cout << "perpendicular: " << perpendicular << std::endl;
  
}

void ProjectVectorAbsolute(Eigen::Vector2d vec_a2, Eigen::Vector2d vec_b2, 
  double& parallel, double& perpendicular ){

  Eigen::Vector3d vec_a, vec_b;
  vec_a << vec_a2[0], vec_a2[1], 0;
  vec_b << vec_b2[0], vec_b2[1], 0;

  /* rotation from a to b (Kosinussatz) */
  double phi = acos( vec_a.dot(vec_b) / (vec_a.norm()*vec_b.norm()) );

  Eigen::Vector3d cross_product = vec_a.cross(vec_b);
  double sign = 1;
  if( cross_product[2] < 0) {sign = -1;}

  parallel = cos(phi)*vec_b.norm();
  perpendicular = sign*sin(phi)*vec_b.norm();
  std::cout << std::endl << "---------------" << std::endl;
  std::cout << "phi          : " << phi << std::endl;
  std::cout << "parallel     : " << parallel << std::endl;
  std::cout << "perpendicular: " << perpendicular << std::endl;

}