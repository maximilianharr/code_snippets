/**
 *  @file cpp_random_engine.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 30.07.2017
 *
 *  @brief Time management
 *         https://www.youtube.com/watch?v=NBogC7V9-60&index=17&list=PL5jc9xFGsL8FWtnZBeTqZBbniyw0uHyaH
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
#include <sstream>

#include <random>
#include <chrono>

// PROJECT INCLUDES
 
// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

using namespace std;

void printRandom(string str, default_random_engine e){
  cout << str;
  for (int i = 0; i < 10; i++){
    cout << e() << " ";
  }
  cout << endl;
}

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{

  /* Note: Random engines on computers are always pseudo-random ! 
   * 
   */
  default_random_engine eng;
  cout << "Min of engine: " << eng.min() << endl;
  cout << "Max of engine: " << eng.max() << endl;
 
/* Note: Random engines create random numbers depending on their current state / number */
  stringstream state;
  state << eng;
  cout << eng() << endl; // eng() is a functor
  cout << eng() << endl;

  state >> eng; // Restore engine state
  cout << eng() << endl; // will print the same output as above
  cout << eng() << endl;

  /* Create engines with / without a seed */
  unsigned seed = chrono::steady_clock::now().time_since_epoch().count(); // random seed taken from clock
  default_random_engine eng1;
  default_random_engine eng2;
  default_random_engine eng3(seed);

  printRandom("eng1: ", eng1);
  printRandom("eng2: ", eng2);
  printRandom("eng3: ", eng3);

  /* There are 16 random engines
   * default_random_engine, minstd_rand, mt19937, mt19937_64, ... 
   */
  mt19937 eng4(seed);

  /* There are various distributions (samples need to generated from random engine)
   * uniform_real_distribution, bernoulli_distributino, exponential_distribution, gamma_distribution, weibull_ ...
   */  
  uniform_int_distribution<int> distr(0,5); // range : [0, 5]
  poisson_distribution<int> distrP(1.0); // mean
  normal_distribution<double> distrN(10.0, 3.0); // mean, std
  cout << "uniform: " << distr(eng4) << endl;
  cout << "poisson: " << distrP(eng4) << endl;
  cout << "normal : " << distrN(eng4) << endl;

  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


