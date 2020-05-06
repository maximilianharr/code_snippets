/**
 *  @file container_iterators.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 15.07.2017
 *
 *  @brief Iterators on cstdlib containers
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
#include <vector>
#include <string>
#include <algorithm>

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES
template <class T>
void PrintVec(std::string note, std::vector<T>& vec){
  std::cout << std::endl << note << std::endl;
  for( auto it = vec.begin(); it != vec.end(); ++it)
    std::cout << *it << " ";
  std::cout << std::endl;
}

bool equalsThree(int i){
  if (i==3)
    return true;
  else
    return false;
}

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  // https://www.youtube.com/watch?v=vO2AlrBf5rQ

  /* Find minimal value in container using cstdlib algorihm */
  std::vector<int> vec = { 4, 2, 5, 1, 3, 9};
  std::vector<int>::iterator it_min = std::min_element(vec.begin(), vec.end());
  std::cout << *it_min << std::endl;

  /* Sort vector from begin to minimum element 
   * Note: std containers do not include last element [begin, end) */
  std::sort(vec.begin(), it_min);
  PrintVec("vec after sorting:",vec);

  /* Copy content from it_min to end of vec (overwriting)
   * Note that vec2 has to be big enough to store the data from vec! */
  std::vector<int> vec2(2); // at least 3 
  std::copy(it_min, vec.end(), vec2.begin());
  PrintVec("copied content from vec in vec2:", vec2);

  /* Alternative to copy (which is efficient) is back_inserter (not efficient).
   * Best is vectors insert()-function */
  std::vector<int> vec3;
  vec3.insert(vec3.end(), it_min, vec.end());
  PrintVec("Inserted data from vec in vec3:", vec3);

  /* Use a defined funtion to check if a vector entries specify certain constraints */
  std::vector<int>::iterator it_3 = find_if(vec.begin(), vec.end(), equalsThree);

  if(it_3 != vec.end())
    std::cout << "Found 3: " << *it_3 << std::endl;

  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


