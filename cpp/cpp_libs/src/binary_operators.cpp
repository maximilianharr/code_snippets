/**
 *  @file eigen_map.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 24.02.2020
 *
 *  @brief Comparing binary operators
 *
 *  @bug
 *  @todo
 */

// PRAGMA

// SYSTEM INCLUDES
#include <iostream>  /* Header that defines the standard input/output stream objects */

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  bool result = true ^ true;
  std::cout << "true ^ true: " << result << std::endl;

  result = true ^ false;
  std::cout << "true ^ false: " << result << std::endl;

  result = false ^ false;
  std::cout << "false ^ false: " << result << std::endl;

  result = false ^ true;
  std::cout << "false ^ true: " << result << std::endl;

  result = false ^ true  ^ true ^ false;
  std::cout << "false ^ true ^ true ^ false: " << result << std::endl;

  result = false ^ true  ^ false ^ true;
  std::cout << "false ^ true ^ false ^ true: " << result << std::endl;

  result = false ^ true  ^ false ^ false;
  std::cout << "false ^ true ^ false ^ false: " << result << std::endl;

  result = true ^ false  ^ true ^ true;
  std::cout << "true ^ false ^ true ^ true: " << result << std::endl;

  result = true ^ true ^ false ^ true;
  std::cout << "true ^ true ^ false ^ true: " << result << std::endl;

  int result2 = true + true + false + true;
  std::cout << "true + true + false + true: " << result2 << std::endl;
}
