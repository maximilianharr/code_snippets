#include <iostream>
#include <signal.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sstream>

/* g++ test.cpp && ./a.out */

int main(int argc, char** argv)
{
  int kf_mode = 0;
  std::string node_opel_ns = "/sensor/opel";

  std::ostringstream oss;

  oss << node_opel_ns << "/kf_pose_" << kf_mode;
  std::cout << oss.str() << std::endl;
  return EXIT_SUCCESS;
}
