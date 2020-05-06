/**
 *  @file cpp_ptr.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 21.11.2016
 *
 *  @brief Investigation of smart pointers.
 *         Smart pointers mimic "normal" pointers (by means of operator overloading),
 *         but furthermore provide additional memory management features (deletion, ...).
 *         They are intended to reduce bugs caused by misuse of pointers.
 *         https://en.wikipedia.org/wiki/Smart_pointer
 *         https://msdn.microsoft.com/de-de/library/hh279676.aspx 
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
#include <iostream> /* Defines the standard input/output stream objects */
#include <cstdlib> /* Defines several general purpose functions */
#include <memory> /* Defines smart pointers (unique_ptr, shared_ptr, ...) */

#include <boost/scoped_ptr.hpp>

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

using namespace std;

class myData{
  public:
  int x,y;
  myData(){};
};

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{

// RAW POINTER
 /* This is a standard C++ pointer which has to be explictly deleted.
  * Other than eg. a class instance (which can be pointed to) which automatically gets deleted */
  int* r_ptr0 = new int (5); // raw pointer (has to be deleted)
  *r_ptr0 = 5;
  delete r_ptr0;

// SMART POINTERS
/* See below: unique_prt, shared_ptr, weak_ptr, scoped_ptr 
 * When to use smart pointers: https://stackoverflow.com/questions/6675651/when-should-i-use-raw-pointers-over-smart-pointers 
 */

// UNIQUE POINTER (introduced in C++11)
/* Properties:
 * - prevents copying of its contained pointer 
 * - std::move can transfer ownership to another unique_ptr
 * - Cannot be copied (copy constructor and assignment operators explicitly deleted)
 * - ONLY ONE OWNER PER UNIQUE_PTR is allowed
 */

  /* Create unique int-pointer */
  unique_ptr<int> ptr0 = make_unique<int>();

  ptr0 = make_unique<int>(4); // set element to 4. Also works with *ptr0 = 4;
  int a1 = 1;
  int *p1, *p2;
  /* Store address of int a1 in pointer p1 works. Can be copied in pointer p2 */
  p1 = &a1;
  p2 = p1;
  /* The address of ptr0 cannot be copied > error. But the data can be copied in int a1 */
  //  p2 = ptr0; // compile error
  a1 = *ptr0;
  printf("Value of c1: %d \n",*p2);
  printf("Value of ptr0: %d \n", *ptr0);

  /* Ownership of contained pointer (ptr0) can be moved to another unique_ptr (ptr1) */
  unique_ptr<int> ptr1 = std::move(ptr0);
  printf("Value of ptr1: %d \n", *ptr1);

  /* unique_ptr can also be used for a class or array */
  unique_ptr<myData> my_array = make_unique<myData>();
  unique_ptr<int[]> my_int = make_unique<int[]>(5);

  for (unsigned int iter_i = 1; iter_i < 5; iter_i++){
    my_int[iter_i] = iter_i;
  }
  printf("Int-value %d \n",my_int[3]);

  /* Release ownership of ptr0 
   * unique_ptr gives up ownership of raw pointer and WILL NOT be delted therefore
   * get() instead of release() and does not give up ownership */
  int* p = ptr0.release();
  delete p; // deletion required as ownership has been released to raw pointer
  

// SHARED POINTER  
/* Properties:
 * - counting ownership of its contained pointer 
 * - will be destroyed when all copies of the shared_ptr have been destroyed
 */
  int* val = new int (5); // necessary to use new to ensure that shared_ptr can free memory and is not done automatically
  std::shared_ptr<int> s_ptr_do_not_do_this( val ); // operator new allocates dynamic memory ( http://www.cplusplus.com/doc/tutorial/dynamic/ )
  std::shared_ptr<int> s_ptr_do_not_do_this2( val );
  s_ptr_do_not_do_this.reset(); // s_ptr = nullptr; does the same
  // now s_ptr_do_not_do_this2 is deleted !

  // USE MAKE_SHARED ! [ https://www.youtube.com/watch?v=qUDAkDvoLas&spfreload=1 ]
  /* Note: Raw pointer (as val above) should not be used and then linked to shared_ptr
   * as anouther shared_ptr may remove the object. Faster and safer is: */
  std::shared_ptr<int> s_ptr0 = make_shared<int>(5);

  std::shared_ptr<int> s_ptr1 = s_ptr0; //Both now own the memory.
  *s_ptr0 = 10;
  std::cout << *s_ptr1 << std::endl; 

 /* Caution: Not initializing with new results in double free or corruption error
  * Since shared_ptr frees memory but p also frees memory. Initialize with NEW ! 
  * int p[5];
  * std::shared_ptr<int> a (p);
  */

  /* Using custom deleters for shared_ptr */
  shared_ptr<int> s_ptr2 (new int[3]); // s_ptr2 points to int[0], so int[1] and int[2] will NOT be deleted!
  shared_ptr<int> s_ptr3 (new int[3], 
                          [](int* p){delete[] p;}); // lambda function destructor of shared_ptr
 
  /* Get raw pointer of shared_ptr */
  int* raw_ptr = s_ptr2.get(); // ONLY do this if you know what you are doing ! unique_ptr has release() instead of get()

// WEAK POINTER
/* Properties:
 * - copy of a shared_ptr
 * - existence / destruction of weak_ptr have no effect on the shared_ptr
 * - After copies of shared_ptr have been destroyed, all weak_ptr copies become empty.
 */
 std::weak_ptr<int> w_ptr = s_ptr0; // s_ptr0 still owns the memory

 /* Weak pointer is invalid when all instances of shared_ptr on val are resetted */
 s_ptr0.reset();
 std::cout << "weak_pointer: " << std::endl << w_ptr.expired() << std::endl;
 s_ptr1.reset();
 std::cout << "weak_pointer: " << std::endl << w_ptr.expired() << std::endl;

// BOOST SCOPED POINTER (https://stackoverflow.com/questions/106508/what-is-a-smart-pointer-and-when-should-i-use-one)
/* Properties:
 * - Can not be copied (because it would prevend it from beeing deleted)
 * - Pass by reference to other functions is possible
 */
  int* bs_val = new int (111);
  {
    boost::scoped_ptr<int> bs_ptr(bs_val);
    std::cout << "bs_val: " << *bs_val << std::endl;
  } /* boost::scopted_ptr goes out of scope -- 
     * bs_val is destroyed */
  // std::cout << "bs_val: " << *bs_val << std::endl; // undefined behavior, bs_value is 0 or random number
  
  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////
