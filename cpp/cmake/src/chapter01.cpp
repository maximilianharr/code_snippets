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
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <string> 
#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>
 
// PROJECT INCLUDES
#include <effective_modern_cpp_config.h>

// LOCAL INCLUDES
#ifdef USE_MYLIB
#include <MyLib.h>
#endif

// FORWARD REFERENCES

// FUNCTION PROTOTYPES
struct student{
  int id;
  std::string name;
  int tm_year;
  int tm_mon;
  int tm_mday;
  int tm_hour;
  int tm_min;
  int tm_sec;
};

static void dump_student(struct student *st){
  fprintf(stdout, "id: %i\n name: %s\n time: %i.%i.%i | %i:%i:%i\n\n", st->id, st->name.c_str(), st->tm_mday, st->tm_mon, st->tm_year, st->tm_hour, st->tm_min, st->tm_sec);
}

using namespace std;
using namespace Eigen;

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{

  /* Print version of program */
  fprintf(stdout, "\nProgram: %s | Version: %d.%d\n\n", argv[0], effective_modern_cpp_VERSION_MAJOR, effective_modern_cpp_VERSION_MINOR);

  /* std vector */
  vector<double> d_vec;
  d_vec.push_back(18.0);
  d_vec.push_back(4.0);
  d_vec.push_back(1989.0);
  
  /* Use and fill eigen-matrix (This section is used to check pretty printing module for eigen in gdb) */
  MatrixXd mat(3,3);
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  std::cout << "mat is:\n" << mat << "\n\n";

  /* Make shared pointer (used to check pretty printing module for STL in gdb) */
  shared_ptr<int> val1 = make_shared<int>();
  val1 = make_shared<int>(4);
  printf("Value of val1: %d \n", *val1);

  /* Create instace of 'student' */
  struct student st = {
    .id = 123,
    .name = "John Doe",
    .tm_year = 2017,
    .tm_mon = 3,
    .tm_mday = 15,
    .tm_hour = 9,
    .tm_min = 30,
    .tm_sec = 0
  };
  dump_student(&st);

  /* Use library math function */
  #ifdef USE_MYLIB
  for(int iter_i = 0; iter_i < 10; iter_i++)
  {
    double z = mylib::sqrt(iter_i);
    fprintf(stdout, "sqrt() returns: %f \n\n", z);
  }
  #endif
  
  char *str = "foo";
  str[0] = 'b';   // << Segfault here 
  
  /* Check if this line prints in test-procedure */
  fprintf(stdout, "Programm finished successfully.\n");
  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


