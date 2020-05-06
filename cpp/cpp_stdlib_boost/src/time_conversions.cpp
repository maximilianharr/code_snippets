/**
 *  @file eigen_lib.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 12.04.2017
 *
 *  @brief Convert data (eg. ublox) to GPST (ms) for Sensoris (Ko-HAF)
 *         http://www.cplusplus.com/reference/ctime/time/
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

#include <stdio.h>      /* printf */
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <math.h>

int main ()
{
  time_t timer;
  struct tm utc_start = {0};
  struct tm utc_current = {0};
  double seconds;

  utc_start.tm_hour = 0;   
  utc_start.tm_min = 0; 
  utc_start.tm_sec = 0;
  utc_start.tm_year = 70; 
  utc_start.tm_mon = 0; 
  utc_start.tm_mday = 1;

  utc_current.tm_hour = 0;   
  utc_current.tm_min = 1; 
  utc_current.tm_sec = 0;
  utc_current.tm_year = 70; 
  utc_current.tm_mon = 0; 
  utc_current.tm_mday = 1;

  seconds = difftime(mktime(&utc_current),mktime(&utc_start));

  printf ("%.f seconds since January 1, 1970 in the current timezone.\n", seconds);

  // Get floating part of double:
  double param, fractpart, intpart;
  param = 3.14159265;
  fractpart = modf (param , &intpart);
  printf("Pi is %f + %f \n", intpart, fractpart);


  return 0;
}
