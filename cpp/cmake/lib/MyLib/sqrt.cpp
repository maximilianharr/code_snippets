/* Simple library adding two values */

#include "MyLib.h"
#include <stdio.h>
#include <cmath>

double mylib::sqrt(double x)
{
  fprintf(stdout, "Using sqrt from MyLib.\n");

  double result = std::sqrt(x);
  return result;
}
