/**
 *  @file ravlue.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 15.07.2017
 *
 *  @brief rvalue lvalue
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

#include <chrono>
#include <memory>

#include <vector>

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES
int sum( int x, int y){ return x+y; }

void square(int& x){ x = x*x; }

void printInt(int& i) { std::cout << "lvalue reference: " << i << std::endl;} // lvalue
void printInt(int&& i){ std::cout << "rvalue reference: " << i << std::endl;} // rvalue
// void printInt(int i ){} // invalid function because compiler sees this as ambiguous

void PrintDeltaTime(std::string str, std::chrono::time_point<std::chrono::system_clock>& t_start){
  std::chrono::time_point<std::chrono::system_clock> t_end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = t_end - t_start;
  std::cout << str << elapsed_seconds.count() << "s\n";
  t_start = t_end;
}

class boVector{

  public:
  uint64_t size; // 1 MB
  double* arr_; // big array

  boVector(int size_in){ // Constructor
    size = size_in;
    arr_ = new double[size];
    for (int i = 0; i < size; i++){
      arr_[i] = i;
    }
  }
  boVector(const boVector& rhs){ // Copy constructor (computational expensive)
    size = rhs.size;
    arr_ = new double[size];
    for (int i=0; i<size; i++){ arr_[i] = rhs.arr_[i]; }
  }
  boVector(boVector&& rhs){ // Move constructor (rvalue function, almost as efficient as call by reference)
    size = rhs.size;
    arr_ = rhs.arr_;
    rhs.arr_ = nullptr; // no const since rhs will be altered
  }
  ~boVector() { delete[] arr_;}
  private:
};

void bovector_by_val(const boVector bo_vector){
  // Do something (bo_vector can not be changed)
}
void bovector_by_ref(const boVector& bo_vector){
  // Do something (bo_vector can not be changed)
}
void vector_by_val(const std::vector<int> bo_vector){
  // Do something (bo_vector can not be changed)
}
void vector_by_lr_val(const std::vector<int>& bo_vector){
  // Do something (bo_vector can not be changed)
  std::cout << "Calling foo with rvalue" << std::endl;
}
void vector_by_lr_val(const std::vector<int>&& bo_vector){
  // Do something (bo_vector can not be changed)
  std::cout << "Calling foo with lvalue" << std::endl;
}
void vector_by_shared_ptr_val(const std::shared_ptr< std::vector<int> > vec){
  // Do something
}
void vector_by_shared_ptr_ref(const std::shared_ptr< std::vector<int> >& vec){
  // Do something
}

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{

  /* Simplified Definition:
   * lvalue: An object that occupies some identifiable location in memory (most variables)
   * rvalue: Any object that is not a lvale 
   * https://www.youtube.com/watch?annotation_id=annotation_1694685217&feature=iv&src_vid=IOkgBrXCtfo&v=UTUdhjzws5g 
   */
  int i; // lvalue [ has location &i ]
  int x = 2; // 2 is an rvalue [ &2 returns error ]
  int y = (2+i); // (2+i) is an rvalue [ &(i+2) returns error ]
  
  int xy = sum(3, 4); // lvalues: 3,4 | rvalues: xy
  
  square(x);  // works, x is an lvalue
  // square(40); // fails, 40 is an rvalue

  int z = xy; // xy is transformed into an rvalue for assignment (but rvalue cannot be transformed into lvalue)

  /* C++11 introduces rvalue reference 
   * 1. Moving semantics
   * 2. Perfect forwarding
   */
   printInt(z); // calling the lvalue function
   printInt(6); // calling the rvalue function
   printInt( std::move(z) ); // rvalue function, z moved to function and should NOT be used anymore
   /* Calling the rvalue function has performance benefits as there will not be a copy created (see below)
    * Note that with rvalue overloading we can use the same function name printInt for lvalue and rvalue */

  std::chrono::time_point<std::chrono::system_clock> t_start;
  t_start = std::chrono::system_clock::now();

  /* Init a huge vector */
  uint64_t size = 1e7;
  std::cout << std::endl << "--- bovector class ---" << std::endl << std::endl;
  boVector bo_vector(size); // Init a huge vector
  PrintDeltaTime("Initiliazing boVector:          ", t_start);
  /* Call by value
   * Will make copy > Computational expensive */
  bovector_by_val( bo_vector );
  PrintDeltaTime("Calling bovector by value:      ", t_start);
  /* Call by reference
   * Will not make a copy > Computational inexpensive, but will alter the object */
  bovector_by_ref( bo_vector );
  PrintDeltaTime("Calling bovector by reference:  ", t_start);
  /* Call by copy constructor 
   * Will copy each element > Computational expensive */
  boVector bo_vector_copy(bo_vector);   
  PrintDeltaTime("Copying boVector:               ", t_start);
  /* Call by move constructor 
   * Will not make a copy > Computational inexpensice, but bo_vector is now nullptr 
   * Thus calling foo_by_val( bo_vector ) again will segfault !*/
  boVector bo_vector_move( std::move(bo_vector) ); 
  PrintDeltaTime("Moving boVector:                ", t_start);
  /* Results: (how much is calling by reference/std::move faster?)
   * size = 1e0 : ~equal
   * size = 1e2 : ~2-10 times faster
   * size = 1e4 : ~10 times faster
   * size = 1e6 : ~1e2 times faster
   * size = 1e8 : ~1e4 times faster
   * size = 1e9 : ~1e5 times faster
   */

  std::cout << std::endl << "--- std::vector class ---" << std::endl << std::endl;
  std::vector<int> vec;
  for( int i = 0; i < size; i++){
    vec.push_back(i);
  }
  PrintDeltaTime("Initiliazing std::vector:       ", t_start);

  vector_by_val( vec );
  PrintDeltaTime("Calling std::vector by value:   ", t_start);

  auto s_ptr_vec = std::make_shared< std::vector<int> >(vec);
  vector_by_shared_ptr_val(s_ptr_vec);
  PrintDeltaTime("Calling shared_ptr by val:      ", t_start);
  vector_by_shared_ptr_ref(s_ptr_vec);
  PrintDeltaTime("Calling shared_ptr by ref:      ", t_start);
  vector_by_lr_val( vec );
  PrintDeltaTime("Calling std::vector by lvalue:  ", t_start);
  vector_by_lr_val( std::move(vec) );
  PrintDeltaTime("Calling std::vector by rvalue:  ", t_start);

  /* Notes on using rvalue reference:
   * - Move constructor (with std::move) ONLY IF both passing by value and reference is needed
   * - If object always passed by reference then Move Semantics are NOT NECESSARY
   * - std::moves allows finer control of which part of your object to be moved
   */
  

  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


