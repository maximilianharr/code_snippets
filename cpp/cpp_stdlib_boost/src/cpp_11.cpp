/**
 *  @file cpp11.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 18.07.2017
 *
 *  @brief Investionation of C++11 standard.
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
#include <cstdlib>
#include <typeinfo>
#include <cassert>

#include <vector>
#include <initializer_list>

#include <regex> // for matching patterns

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


class boVector{
  public:
  int a = 9; // In-class initialization also possible in C++11
  std::vector<int> vec0;
  
  /* Iterate through all elements of initializer and push into std::vector */
  boVector() { /* Delegating constructor */
    std::cout << "Created boVector with a = " << a << std::endl;
  }
  boVector(const std::initializer_list<int>& v) : boVector() {
    for( std::initializer_list<int>::iterator itr = v.begin(); itr != v.end(); ++itr ){
      vec0.push_back(*itr);
    }
  }
  
  /* override */
  virtual void fun01(int){};
  virtual void fun02(float){};

  /* delete */
  boVector& operator=(const boVector&) = delete; // explictly remove copy operator (automatically implemented by compiler)
  boVector(int input) { a = input; }
  boVector(double) = delete; // Prevent initialzation with double (over int-constructor) by explictly deleting this constructor
};

class boVectorChild : public boVector{
  //virtual void fun01(float) override; // Compile error: This function does not exist in parent > Makes code more type safe
  virtual void fun02(float) override {}; // Compiles, function will be overwritten
};

/* constexpr tells compiler that function will always return constant expression
 * This might be useful as constant expressions are sometimes needed (eg. initializing an array)
 * Function output will be computed at compile time! > No computation time at runtime! */
constexpr int ReturnConstInt(int c){
  int a = 6;
  int b = 4;

  return ( a + b ) / 2 * c ;
}

template <typename func> // func is a lambda function (see below)
void my_filter(func f, std::vector<int> arr){
  for (auto i: arr){ // nice for-loop for vector elements (instead of iterator)
    if (f(i)){
      std::cout << i << " "; }

  }
}

/* User defined literals
 * https://www.youtube.com/watch?v=2xUJTXeBZmE&index=5&list=PL5jc9xFGsL8FWtnZBeTqZBbniyw0uHyaH
 * constexpr will make it be computed at compile (and not run) time
 */
constexpr long double operator "" _m(long double x){ return x*1;}
constexpr long double operator "" _cm(long double x){ return x/100;}
constexpr long double operator "" _mm(long double x){ return x/1000;}


//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
// https://www.youtube.com/watch?v=U6mgsPqV32A&list=PL5jc9xFGsL8FWtnZBeTqZBbniyw0uHyaH&index=1

 /* Uniform initialization */
  std::vector<int> vec = {1, 2, 4, 9, 16}; // calls initializer_list (iplemented for all stdlib containers)
  PrintVec("Initialized std::vector", vec);

  /* Define your own initializer list */
  boVector v = {1, 2, 4, 9, 16};
  PrintVec("initialized boVector: ", v.vec0);

  /* auto type */
  auto a = 1; // will be an int
  std::cout << "variable a is of type: " << typeid(a).name() << std::endl << std::endl;

  /* foreach */
  std::cout << "Simple foreach loop accessing elements of std::vector: " << std::endl;
  for (int i : vec){
    std::cout << i << " ";
  }
  std::cout << std::endl;

  /* nullptr 
   * So far NULL could be both a NULL-object or pointer. nullptr is a NULL-pointer and removes ambiguity */

  /* enum class */
  enum class apple {green, red};
  enum class orange {green, red};
  apple apple0 = apple::red;
  orange orange0 = orange::red;

  /* if( apple0 != orange0 ) 
   * Compile fails since ==( apple, orange ) has not be defined !
   * would have been true if it was a simple enum where apple::red is and integer */

  /* static_assert
   * assert() known from prior C++ versions asserts if condition is not true
   * static_assert() makes assertion DURING COMPILE TIME */
  int* ptr0 = &a;
  const int value0 = 2;
  assert( a > 0 ); // Does not assert since true
  static_assert( value0 > 1, "invalid" ); // Does not assert during compilation since true | value0 needs to be constant!

  /* Delegating constructor (see boVector-class)
   * The default constructor can be called when calling another constructor > no init()-function necessary */

  /* overide (see boVectorChild)
   * virtual functions of parents can be overwritten as virtual function in child */

  /* delete 
   * Delete specific function that are usually create by compiler (eg =operator, constructor, ... ) */

   /* constexpr - Tell compiler that function returns constant expression
    * This forces the compiler to compute output at compile time > No computation time during runtime! */
   int myarray[ 4 + ReturnConstInt(11) ]; // return type is constexr

  /* New string literals */
  std::string str = u8"string"; // UTF-8 string

  /* lambda function (direct access) 
   * Very handy when it comes to functional programming where eg. an filter uses function handle and input value
   * eg. stdlib functions for_each, .. or my_filter (see below) */
  std::cout << std::endl << "output of lambda function: " << 
    [](int x, int y){return x+y;}(3,4) << std::endl; // (3,4) are input of function
 
  /* Store lambda function and then execute */
  auto lambdafun = [](int x, int y){ 
    x++;
    return x+y;
  };
  std::cout << "Lambda function +1: " << lambdafun(3,4) << std::endl;

  /* Using with lambda function with functor */
  std::cout << std::endl << "Outputting elements of vec >3: ";
  int y = 3;
  // [&] tells compiler that we want "variable capture" > use variable "y" from outside
  my_filter([&](int x){ return x > y;}, vec); 
  std::cout << std::endl;

  /* User defined literals */
  long double height = 1.35_m;
  std::cout << "1.35 m + 24 cm: " << (height + 24.0_cm) << " m" << std::endl;

  /* Regular expression (standardized way to express patterns to be matched against sequences of characters)*/
  std::string str0 = "hello";

  std::regex ex1("Hello");
  bool match = regex_match(str0, ex1);
  std::cout << (match? "match" : "no match") <<  std::endl;
  
  std::regex ex2("Hello", std::regex_constants::icase); // see http://www.cplusplus.com/reference/regex/regex_constants/
  match = regex_match(str0, ex2);
  std::cout << (match? "match" : "no match") << std::endl;

  std::regex ex3("Hello??", std::regex_constants::icase); // ? . * + [] ^ |  \ () :
  match = regex_match(str0, ex3);
  std::cout << (match? "match" : "no match") << std::endl;

  match = regex_search("Mark: Hello stranger", ex2);
  std::cout << (match? "match" : "no match") << std::endl;
 
  /* Regular expression grammars: ECMAScript, basic, extended, awk, grep, egrep 
   * regex_constants::grep switches grammar/search keywords to unix "grep" (command line tool) syntax
   * https://www.youtube.com/watch?v=nkjUpUu3dFk&list=PL5jc9xFGsL8FWtnZBeTqZBbniyw0uHyaH&index=13 
   * https://www.youtube.com/watch?v=_79j_-2xMrQ&index=14&list=PL5jc9xFGsL8FWtnZBeTqZBbniyw0uHyaH */
  std::regex ex4("abc.+", std::regex_constants::grep);
  match = regex_search("hi abcx+ there", ex4);
  std::cout << (match? "match" : "no match") << std::endl;

  std::string str2 = "maximilian.harr@daimler.com; david.augustin@gm.com; jeremias.schucker@gmail.com;";
  std::regex ex5("([[:w:]]+)@([[:w:]]+)\.com");
  std::cout << regex_replace(str2, ex5, "$1 is on $2 | ", std::regex_constants::format_no_copy);
  std::cout << std::endl;

  return 0;

}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////
