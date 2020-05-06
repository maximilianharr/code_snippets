#include <cmath>
#include <vector>
#include <map>
#include <iostream>
#include <chrono>
#include <omp.h>

/**
 * @brief print message and elapsed time
 * @param msg string message to be printed
 * @param start chrono time stamp
 */
void printTimeWithInfo(const std::string& msg,
                       std::chrono::time_point<std::chrono::system_clock>& start) {
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << msg << " | Elapsed time: " << elapsed_seconds.count() << "s" << std::endl;
  start = std::chrono::system_clock::now();
}

void complex_computation( double value){
  for(int i = 1; i < 1e3; i++){
    double root = sqrt(value);
  }
}

int main()
{
  const int size = 256;
  double sinTable[size];

  /* Create big std::map */
  std::map<int, double> a;
  for(int i = 1; i < 1e6; i++){
    a.insert(std::make_pair(i,3.141592653589793238462643383279502884197169399375105820974944592307816406286));
  }

  std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
  // #pragma omp parallel for
  for(auto it = a.begin(); it != a.end(); ++it){
    complex_computation(1.0);
  }
  printTimeWithInfo("No parallel:", start);

  #pragma omp parallel for
  for (int i = 0; i < a.size(); i++) {
    /* code */
    complex_computation(1.0);
  }
  printTimeWithInfo("Parallel 1:", start);

  #pragma omp parallel
  {
      size_t cnt = 0;
      int ithread = omp_get_thread_num();
      int nthreads = omp_get_num_threads();
      for(auto element = a.begin(); element !=a.end(); ++element, cnt++) {
          if(cnt%nthreads != ithread) continue;
          //foo(element->first, element->second);
          complex_computation(element->second);
      }
  }
  printTimeWithInfo("Parallel 2:", start);

}
