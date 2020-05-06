/**
 *  @file chapter01.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 23.02.2017
 *
 *  @brief DEDUCING TYPES
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

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES


//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{

  int n = 2;

  /* Get user input */
  if( argc == 2){
    n = atoi( argv[1] );
  }
    
  std::cout << "Hello World " << n << "\n";
  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


