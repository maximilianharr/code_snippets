/**
 *  @file create_and_manage.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 29.11.2017
 *
 *  @brief Create and manage boost::threads
 *         See: https://theboostcpplibraries.com/boost.thread-management
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *
 *  @bug attrs.set_stack_size(4096*10) does not work
 *
 *
 *  @todo
 *
 *
 *
 */

#include <boost/thread.hpp>
#include <boost/thread/scoped_thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>

namespace test {

/** @brief Function to be executed by thread
 *  @param
 *  @return
 */
void func01(int sleep_time, int max_i)
{
  // boost::this_thread::disable_interruption no_interruption; // This line would disable thread interruption
  try{
    for (int i = 0; i < max_i; ++i)
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds{sleep_time});
      std::cout << i << std::endl;
    }
  }
  catch( boost::thread_interrupted& ){
    std::cout << "Caution, thread interrupted: " << boost::this_thread::get_id() << std::endl;
  };
}

} // end of namespace test

int main()
{
  /* Get thread information
   * get_id()
   *   returns a number to uniquely identify the current thread
   * hardware_concurrency()
   *   returns the number of threads that can physically be executed at the same time*/
  std::cout << boost::this_thread::get_id() << std::endl;
  std::cout << boost::thread::hardware_concurrency() << std::endl;

  std::cout << "---- THREAD 1 ----" << std::endl;
  boost::thread t1{ boost::bind(&test::func01, 50, 5) };
  /* join() blocks the current thread (main) until the thread for which
   * join() was called (thread t) has terminated. */
  t1.join();

  /* interruption points make it possible to interrupt threads */
  std::cout << "---- THREAD 2 ----" << std::endl;
  boost::thread t2{ boost::bind(&test::func01, 50, 5) };
  boost::this_thread::sleep_for(boost::chrono::milliseconds{120});
  t2.interrupt();
  t2.join();

  /* Setting thread attributes */
//  std::cout << "---- THREAD 3 ----" << std::endl;
//  boost::thread::attributes attrs;
//  attrs.set_stack_size(4096*10); // @todo : does not work > fix?
//  boost::thread t3{attrs, boost::bind(&test::func01, 50, 5) };
//  t3.join();

  /* The results of tx will not be shown as this thread (main) finishes before t3 does */
  std::cout << "---- THREAD X ----" << std::endl;
  boost::thread tx{ boost::bind(&test::func01, 50, 5) };
}
