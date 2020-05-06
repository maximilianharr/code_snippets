/**
 *  @file tests.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 07.04.2017
 *
 *  @brief Template for testing library with gtest
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

// SYSTEM INCLUDES
#include <gtest/gtest.h>

// PROJECT INCLUDES

// LOCAL INCLUDES
#include "whattotest.hpp"

// TEST FUNCTIONS
/* Ordinary C++ function without return value i
 * Result is determined by assertions. If any fail > entire test fails
 */
TEST(SquareRootTest, PositiveNos) {
    ASSERT_EQ(6, mylib::squareRoot(36.0)) << "Output is invalid.";
}

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {

  /* Parse command line for Google Test flags (eg. when using --gtest_output, gtest_filter, .. */
  testing::InitGoogleTest(&argc, argv);

  /* Run tests */
  return RUN_ALL_TESTS();
}

