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

/** @brief Standard command line parameter processing.
 *  @param Pass command line parameters
 *  @return 0 if -h flag is set
 */
int cmd_check(int, char*[]);

// GLOBAL VARIABLES


using namespace Eigen;
using namespace std;

void outputAsMatrix(const Eigen::Quaterniond& q)
{
    std::cout << "R=" << std::endl << q.normalized().toRotationMatrix() << std::endl;
}

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  /* Check command line options. Stop execution if -h is set. */
  if(!cmd_check(argc,argv)) return 0;

  /* Specify roation axis and angle */
  Eigen::Vector3d rotation_axis{0, 0, 1};
  Eigen::Vector3d pose{0, 0, 1};
  double angle = 0;
  if(argc == 2){
    angle = atof(argv[1]);}
  else{
    angle = M_PI/8.0;
  }

  /* Compute Quaternion by hand */
  auto sinA = std::sin(angle / 2);
  auto cosA = std::cos(angle / 2);
  Eigen::Quaterniond q;
  q.x() = rotation_axis(0) * sinA;
  q.y() = rotation_axis(1) * sinA;
  q.z() = rotation_axis(2) * sinA;
  q.w() = cosA;

  /* Let Eigen compute Quaternion (use Vector3d::UnitZ() instead of rotation_axis) */
  Eigen::Quaterniond q2 = Eigen::Quaterniond{ Eigen::AngleAxisd{ angle, rotation_axis } };

  /* Compare rotation matrices */
  outputAsMatrix( q  );
  outputAsMatrix( q2 );

  /* Get rotation matrix from quaternion */
  Matrix3d m; 
  m = Eigen::Quaterniond(q2); 
  std::cout << "Quaternion q2: " << std::endl << q2.x() << std::endl
            << q2.y() << std::endl << q2.z() << std::endl << q2.w() << std::endl;
  std::cout << "Rotation matrix m of Quaternion q2: " << std::endl << m << std::endl;

  /* Multiplication of quaternions */
  Matrix3d m2;
  m2 = Eigen::Quaterniond(q2*q2);
  std::cout << "m*m: " << std::endl << m*m << std::endl;
  std::cout << "Rotation matrix of q2*q2: " << std::endl << m2 << std::endl;

  /* Dividing Quaternions */
  Matrix3d m3;
  m3 = Eigen::Quaterniond(q2.inverse());
  std::cout << "m.inverse(): " << std::endl << m.inverse() << std::endl;
  std::cout << "Rotation matrix of q2.inverse(): " << std::endl << m3 << std::endl;

  /* Rotate Vector */
  Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX();
  std::cout << "q2*x_axis: " << std::endl << q2*x_axis << std::endl;
  
  /* Get rotation axis of Quaternion */
  Eigen::AngleAxisd rot_axis_m(m);
  Eigen::AngleAxisd rot_axis_q2(q2);
  rot_axis_m.axis().cross(rot_axis_q2.axis());
  std::cout << "Rotation axis of m: " << std::endl << rot_axis_m.axis() << std::endl;
  std::cout << "Rotation axis of q2: " << std::endl << rot_axis_q2.axis() << std::endl;

  /* Get Euler Angles from Quaternion */
  Eigen::Vector3d euler = q2.toRotationMatrix().eulerAngles(0,1,2);
  std::cout << "Euler angles yf q2: " << std::endl << euler << std::endl;

  /* Get Quaternion from euler angles */
  Eigen::Quaterniond q3;
  q3 = AngleAxisd(euler[0], Eigen::Vector3d::UnitX())*
      AngleAxisd(euler[1], Eigen::Vector3d::UnitY())*
      AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
  std::cout << "q3 from euler angles: " << std::endl << q3.x() << std::endl
            << q3.y() << std::endl << q3.z() << std::endl << q3.w() << std::endl;
  
  /* Rotate covariance matrix */
  Eigen::MatrixXd covariance_q = Eigen::MatrixXd::Zero(3,3);
  Eigen::Quaterniond q4 = Eigen::Quaterniond{ Eigen::AngleAxisd{ angle, Eigen::Vector3d::UnitZ() } };
  Eigen::MatrixXd covariance_r = Eigen::MatrixXd::Zero(3,3);
  Matrix3d rotation;
  rotation = Eigen::AngleAxisd{ Eigen::AngleAxisd( angle, Eigen::Vector3d::UnitZ()) };

  covariance_q(0,0) = 1;
  covariance_q(1,1) = 2;
  covariance_q(2,2) = 3;
  covariance_r(0,0) = 1;
  covariance_r(1,1) = 2;
  covariance_r(2,2) = 3;

  covariance_q = q * covariance_q; // Rotate covariance matrix in lane coordinates
  covariance_q = covariance_q*covariance_q.transpose();
  covariance_r = rotation*covariance_r*covariance_r.transpose()*rotation.transpose();

  std::cout << "covariance_q:\n " << covariance_q << std::endl;
  std::cout << "covariance_r:\n " << covariance_r << std::endl;


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


