#include <cmath>
#include <vector>
#include <map>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <chrono>
/* run with
 * g++ container_it.cpp && ./a.out
 */

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

int main()
{

  int container_size = 2e3;

  /* Create big std::map */
  std::map<int, double> a;
  for(int i = 1; i < container_size; ++i){
    a.insert(std::make_pair(i,3.141592653589793238462643383279502884197169399375105820974944592307816406286));
  }

  std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

  printTimeWithInfo("####### MAP ", start);
  for( auto it1 = a.begin(); it1 != a.end(); ++it1){
    for( auto it2 = a.begin(); it2 != a.end(); ++it2){
    double val = it2->second;
    double c = val + val + val + val;
    }
  }
  printTimeWithInfo("for( auto it = a.begin(); it != a.end(); ++it){ ", start);

  for(int i = 0; i < a.size(); i++){
    for(int j = 0; j < a.size(); j++){
      double val = a[j];
      double c = val + val + val + val;
    }
  }
  printTimeWithInfo("for(int i = 0; i < a.size(); i++){ ", start);

  for(int i = 0; i < a.size(); ++i){
    for(int j = 0; j < a.size(); ++j){
      double val = a[j];
      double c = val + val + val + val;
    }
  }
  printTimeWithInfo("for(int i = 0; i < a.size(); ++i){", start);
  for( auto elem1 : a){
    for( auto elem2 : a){
      double val = elem2.second;
      double c = val + val + val + val;
    }
  }
  printTimeWithInfo("for( auto elem : a){ ", start);


  printTimeWithInfo("####### VECTOR ", start);
  std::vector<double> b;
  for(int i = 1; i < container_size; ++i){
    b.push_back(3.141592653589793238462643383279502884197169399375105820974944592307816406286);
  }

  for( auto it1 = b.begin(); it1 != b.end(); ++it1){
    for( auto it2 = b.begin(); it2 != b.end(); ++it2){
      double c= *it2 + *it2;
    }
  }
  printTimeWithInfo("for( auto it = a.begin(); it != a.end(); ++it){ ", start);

  for(int i = 0; i < b.size(); i++){
    for(int j = 0; j < b.size(); j++){
      double c= a[j] + a[j];
    }
  }
  printTimeWithInfo("for(int i = 0; i < a.size(); ++i){ ", start);

  for(int i = 0; i < b.size(); ++i){
    for(int j = 0; j < b.size(); ++j){
      double c= a[j] + a[j];
    }
  }
  printTimeWithInfo("for(int i = 0; i < a.size(); i++){ ", start);
  for( auto elem1 : b){
    for( auto elem2 : b){
      double c= elem2 + elem2;
    }
  }
  printTimeWithInfo("for( auto elem : a){ ", start);

}
