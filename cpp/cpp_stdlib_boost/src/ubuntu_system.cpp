

#include<unistd.h>
#include<iostream>
#include<string>
#include<sstream>
#include<ctime>
#include<iomanip>

using namespace std;

int main(int argc, char** argv)
{

  /* Get and print username */
  std::stringstream username;
  username << "Username: " << getenv("USER");
  cout << "\n" <<  username.str() << "\n\n";

  /* Get and print current time and date */
  std::stringstream time_n_date;
  time_t curr_t;
  struct tm * curr_time;
  curr_t = time(0);
  curr_time = localtime(&curr_t);
  time_n_date << (curr_time->tm_year + 1900)
            << std::setw(2) << std::setfill('0') << (curr_time->tm_mon + 1)
            << std::setw(2) << std::setfill('0') << curr_time->tm_mday << '_'
            << std::setw(2) << std::setfill('0') << curr_time->tm_hour
            << std::setw(2) << std::setfill('0') << curr_time->tm_min
            << std::setw(2) << std::setfill('0') << curr_time->tm_sec;
  std::cout << time_n_date.str() << "\n";


  return EXIT_SUCCESS;
}
