/**
 *  @file functors.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 15.07.2017
 *
 *  @brief Use cstdlib functor
 *
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
#include <string>

#include <vector>
#include <set>

#include <algorithm>
#include <functional> /* std::bind, std::placeholders */

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES
template <class T>
void PrintVec(std::string note, std::vector<T>& vec){
  std::cout << std::endl << note << std::endl;
  for( auto it = vec.begin(); it != vec.end(); ++it){
    std::cout << *it << " ";
  }
  std::cout << std::endl;
}

template <class T>
void PrintSet(std::string note, T& myset){
  std::cout << std::endl << note << std::endl;
  for (auto it = myset.begin(); it != myset.end(); ++it){
    std::cout << *it << " ";
  }
  std::cout << std::endl;
}

class X {
  public:
  void operator()(std::string str){ // This is a functor
    std::cout << "Calling functor X with parameter: " << str << std::endl;
  }
  operator std::string () const{ // Type conversion function (not needed here)
    return "X";
  }
};

class AddValue{ // Adds two and prints value
  int val;
  public:
  AddValue(int j) : val(j) { } // constructor (store value of j in val)
  void operator() (int i){
    std::cout << i+val << std::endl;
  }
};

void MultiplyValue(int i, int k){
  std::cout << i*k << std::endl;
}

int PowValue(int i, int k){
  return std::pow(i,k);
}

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  // https://www.youtube.com/watch?v=shqvSkk8r0M
  /* Create class and call functor */
  X foo;
  foo("Hello");

  /* Call type converstion function that converts class X into std::string */
  std::string str_foo = foo;
  std::cout << "Converted foo to string: " << str_foo << std::endl;

  /* Usage of class funtor */
  AddValue addvalue_instance(2); // init class with val = 2
  addvalue_instance(3);

  /* Iterator through vector and add do computation using AddValue-Functor */
  std::vector<int> vec = {0, 1, 2, 4, 8, 16, 32, 64, 128};
  int x = 5;
  std::cout << std::endl << "Using AddValue functor to print a string with for_each: " << std::endl;
  std::for_each(vec.begin(), vec.end(), AddValue(x));

  /* Build in Functors of cstdlib
   * less, greater, greater_equal, less_equal, not_equal_to, multiplies, ... 
    * std::placeholders::_1 substitutes the first input parameter with the value of the container 
    * back_inserter is a back-insert iterator that inserts elements at the end of the container
    * http://www.cplusplus.com/reference/iterator/back_inserter/?kw=back_inserter
    * front_inserter can't be used since std::vector has no push_front member
    */
  std::vector<int> vec2; // Transform data of vec and store in vec2
  std::transform(vec.begin(), vec.end(),      // source
                 std::back_inserter(vec2),    // destination
                 std::bind( std::multiplies<int>(), std::placeholders::_1, 10 )); // modification
  PrintVec("vec2 with 10-times data of vec: ", vec2);

  std::cout << std::endl << "Multiplying each value of vec and printing: " << std::endl;
  std::for_each( vec.begin(), vec.end(), std::bind(MultiplyValue, std::placeholders::_1, 2) );

  /* Transform a vector using own functions */
  auto f = std::function<int (int,int)>(PowValue); // Create a function
  std::vector<int> vec3;
  std::transform( vec.begin(), vec.end(), std::back_inserter(vec3), std::bind(f, std::placeholders::_1, 2) );
  // std::bind(PowValue, std::placeholders::_1, 2) would have been also possible
  PrintVec("vec3 after PowValue function: ", vec3);
  
  /* LAMBDA FUNCTION (C++11)
   * Write function instantly */
  std::vector<int> vec4;
  std::transform( vec3.begin(), vec3.end(), back_inserter(vec4), [](uint64_t x){ return (x < 20) && (x > 5);} );
  PrintVec("vec3 after range check (5,20) lambda function: ", vec4);

/* Choosing another functor to sort std::set */
  std::set<int, std::less<int> > myset_less = { 3, 1, 5, 4, 2};
  PrintSet("myset with std::less : ", myset_less);

  std::set<int, std::greater<int> > myset_greater = { 3, 1, 5, 4, 2};
  PrintSet("myset with std::greater : ", myset_greater);

  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


