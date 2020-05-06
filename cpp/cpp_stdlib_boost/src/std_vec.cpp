// vector::rbegin/rend
#include <iostream>
#include <vector>

int main ()
{
  std::vector<int> myvector (5);  // 5 default-constructed ints

  for( unsigned int i = 0; i<myvector.size(); i++)
  {
    myvector.at(i) = i;
  }

  std::cout << myvector.at(1) << '\n';
  std::cout << myvector[2] << '\n';
  std::cout << *(myvector.rbegin()+1) << '\n';
  
  return 0;
}
