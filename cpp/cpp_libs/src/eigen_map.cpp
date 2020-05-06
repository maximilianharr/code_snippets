/**
 *  @file eigen_map.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 29.06.2017
 *
 *  @brief Eigen map computations
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

  /* Map array of doubles in Eigen::Matrix3d */
  double array[9];
  for(int i = 0; i < 9; ++i) array[i] = i;
  Eigen::Map< Eigen::Matrix3d> residual_0 = Eigen::Map<Eigen::Matrix3d>(array);
  Eigen::Map< Eigen::Matrix<double, 9, 1> > residual_1 = Eigen::Map< Eigen::Matrix<double, 9, 1> >(array);
  std::cout << "residual_0: " << std::endl << residual_0 << std::endl;
  std::cout << "res 0+0: " << std::endl << residual_0 + residual_0 << std::endl;
  std::cout << "residual_1: " << std::endl << residual_1 << std::endl;

	/* Map Eigen::Matrix3d to array of doubles */
	double array2[9];
	Eigen::Map<Matrix<double,3,3,RowMajor> >(array2,3,3) = residual_0;
	std::cout << "Array2:" << std::endl;	
	for(int i = 0; i < 9; ++i){
		std::cout << array2[i] << " ";
	}
	std::cout << std::endl;

  /* Cast Eigen VectorXd to Matrix of float */
  Eigen::Vector3d vector_0;
  vector_0 << 0,1,2;
  Eigen::Matrix<float, 3, 1> p_m = vector_0.template cast<float>();
  std::cout << "p_m: " << std::endl << p_m << std::endl;


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


