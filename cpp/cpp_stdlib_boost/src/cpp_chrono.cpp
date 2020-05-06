/**
 *  @file chrono.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 15.07.2017
 *
 *  @brief Time management
 *         https://www.youtube.com/watch?v=M13otPgOePs&list=PL5jc9xFGsL8FWtnZBeTqZBbniyw0uHyaH&index=15
 *         https://www.youtube.com/watch?v=RuPfDfLKY04&list=PL5jc9xFGsL8FWtnZBeTqZBbniyw0uHyaH&index=16
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
 *
 *  @bug
 *
 *  @todo
 *
 */

// PRAGMA

// SYSTEM INCLUDES
#include <iostream> /* Defines the standard input/output stream objects */
#include <cstdlib> /* Defines several general purpose functions */

#include <chrono>

// PROJECT INCLUDES
 
// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

using namespace std;

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  /* Chrono has three clocks
   * chrono::system_clock          : Time according to system (not steady!)
   * chrono::steady_clock          : Goes at uniform rate
   * chrono::high_resolution_clock : Provides smallest possible tick period
   */  

  ratio<2,10> r; // c++ ratio object
  cout << r.num << " : " << r.den << endl;
  cout << "system clock perios: " << 
    chrono::system_clock::period::num << "/" << chrono::system_clock::period::den << endl; // 100 ns clock

  /* time span */
  chrono::duration<double, ratio<60,1>> timespan(4.0); 
  chrono::milliseconds timespan_mi; // milliseconds is a default chrono::duration
  timespan_mi = chrono::duration_cast<chrono::milliseconds>(timespan);
  cout << timespan.count() << " minutes "<< endl;
  cout << timespan_mi.count() << " milliseconds "<< endl;

  /* time point 
   * 00:00 Jan 1970 (Coordinated Universal Time - UTC) */
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  cout << tp.time_since_epoch().count() << " sytem_clock periods since 01.01.1970 " << endl;

  auto time_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  double time_utc = time_ms.time_since_epoch().count();
  std::cout << "Time UTC in ms: " << time_utc << std::endl;

  /* time point (steady clock) */
  chrono::steady_clock::time_point start = chrono::steady_clock::now();
  std::cout << "Time for printing something: ";
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::steady_clock::duration d = end - start;
  cout << chrono::duration_cast<chrono::nanoseconds>(d).count() << " nanoseconds" << endl;

  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


