/**
 *  @file eigen_isnan.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 29.06.2017
 *
 *  @brief Check if matrix has nan values
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
#include <Eigen/Core>
#include <iostream>

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

// GLOBAL VARIABLES

using namespace Eigen;
using namespace std;


//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{  
	Eigen::Matrix3d v;
	v << 0, 1, 2, 3, 4, 5, 6, 7, 8;

	if( !isnan(v.array()).isZero(0) || !isinf(v.array()).isZero(0) ){
		std::cout << "Matrix has a nan/inf value." << std::endl;
	}
	else{
		std::cout << "Matrix has no nan/inf value." << std::endl;
	}
	v(0,0) *= 0.0/0.0;
	v(1,1) /= 0.0;
	std::cout << v << std::endl << std::endl;
	std::cout << "isinf:" << std::endl << Eigen::isinf(v.array()) << std::endl;
	std::cout << "isnan:" << std::endl << Eigen::isnan(v.array()) << std::endl;
	if( !isnan(v.array()).isZero(0) ){
		std::cout << "Matrix has a nan/inf value." << std::endl;
	}
	else{
		std::cout << "Matrix has no nan/inf value." << std::endl;
	}

}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


