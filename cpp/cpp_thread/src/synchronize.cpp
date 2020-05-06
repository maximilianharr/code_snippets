/**
 *  @file synchronize.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 29.11.2017
 *
 *  @brief Synchronize boost::threads
 *         See: https://theboostcpplibraries.com/boost.thread-synchronization
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *
 *  @bug
 *
 *
 *  @todo
 *
 *
 *
 */

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>

namespace test {

class Class01{
  public:
    /* mutex is to prevent other threads from taking ownership while a particular thread owns the mutex */
    boost::mutex mutex01_;
    boost::timed_mutex tmutex02_;
    boost::shared_mutex smutex02_;
    boost::condition_variable_any cond01_;

    /** @brief Default constructor
     */
    Class01(){
      param01_ = 0;
      param02_ = 0;
    }

    /** @brief Constructor
     */
    Class01(int param01, int param02){
      param01_ = param01;
      param02_ = param02;
    }

    /** @brief Destructor
     */
    ~Class01(){}

    /** @brief Get parameter param01_
     */
    int get_param01(){

      /* lock_guard automatically calls lock() and unlock() in its constructor and its destructor
       * > make sure resources are released when they are no longer needed */
      boost::lock_guard<boost::mutex> lock{mutex01_};
      std::cout << "Reading param01_" << std::endl;
      boost::this_thread::sleep_for(boost::chrono::milliseconds{200});

      return param01_;
    }

    /** @brief Get parameter param02_
     */
    int get_param02(){

      /* shared_lock can be used if threads only need read-only access to a specific resource. */
      boost::shared_lock<boost::shared_mutex> lock{smutex02_}; // lock a shared_mutex so that others can also share lock it
      std::cout << "Reading param02_" << std::endl;
      boost::this_thread::sleep_for(boost::chrono::milliseconds{100});
      std::cout << "Reading finished" << std::endl;
      return param02_;
    }

    /** @brief Set parameter param01_
     */
    void set_param01(int param01, int id){
      /* Lock mutex when setting parameter */
      mutex01_.lock();
      std::cout << id << " : Setting param01_ " << std::endl;
      boost::this_thread::sleep_for(boost::chrono::milliseconds{100});
      param01_ = param01;
      mutex01_.unlock();
      /* Do other stuff, mutex is unlocked since member variables are not changed anymore */
      boost::this_thread::sleep_for(boost::chrono::milliseconds{300});
      std::cout << id << " : Doing other stuff " << std::endl;
      return;
    }

    /** @brief Set parameter param02_
     */
    void set_param02(int param02, int id){
      /* constructor of unique_lock calls try_lock() instead of lock() */
      boost::unique_lock<boost::shared_mutex> lock_shared{smutex02_}; // lock a shared_mutex uniquely > No one can lock it
      boost::unique_lock<boost::timed_mutex> lock{tmutex02_, boost::try_to_lock};
      /* Check if locking was successful and retry for specified amount of time before failing */
      if (lock.owns_lock() || lock.try_lock_for(boost::chrono::milliseconds{10})){
        std::cout << id << " : Setting param02_ " << std::endl;
        boost::this_thread::sleep_for(boost::chrono::milliseconds{100});
      }
      else{
        std::cout << id << " : Failed to set param02_" << std::endl;
      }
      param02_ = param02;
      return;
    }

    /** @brief Wait until notification on cond01_
     */
    void wait(int id){
      cond01_.wait(mutex01_); // waits until some thread calls notify_all()
      std::cout << "Processing : " << id << std::endl;
      return;
    }

    /** @brief Notify on cond01_
     */
    void notify(){
      cond01_.notify_all(); // waits untill some thread calls wait() and then notifies it
      std::cout << "Nofication sent." << std::endl;
      return;
    }

  private:
    int param01_, param02_;
};

} // end of namespace test

int main()
{
  /* Create a Class01 object */
  std::cout << std::endl << "------------------" << std::endl;
  test::Class01 class_01(10,0);
  std::cout << "#1: param01_ is: " << class_01.get_param01() << std::endl;

  /* Bind member functions with boost
   * http://www.boost.org/doc/libs/1_62_0/libs/bind/doc/html/bind.html */
  boost::bind( &test::Class01::set_param01, boost::ref(class_01), _1, 0 )(5);
  std::cout << "#2: param01_ is: " << class_01.get_param01() << std::endl;

  /* Get and set values */
  boost::thread t_get{ boost::bind(&test::Class01::get_param01, &class_01) };
  boost::thread t_set{ boost::bind(&test::Class01::set_param01, &class_01, 30, 0 ) };
  std::cout << "#3: param01_ is: " << class_01.get_param01() << std::endl;
  t_set.join();
  std::cout << "#4: param01_ is: " << class_01.get_param01() << std::endl;

  /* Iteratively open threads and check if mutex.lock() works
   * All threads open rapidly but set_param01 execution is performed step after step since mutex get locked */
  std::cout << std::endl << "---- MULTI THREAD LOOP ----" << std::endl;
  for( int i = 0; i < 5; i++){
    std::cout << "Open thread: " << i << std::endl;
    boost::thread t_mul{ boost::bind(&test::Class01::set_param01, &class_01, 50, i ) };
  }
  /* if aborted before t_mul is finished:
   * boost::mutex::~mutex(): Assertion `!res' failed. */
  boost::this_thread::sleep_for(boost::chrono::milliseconds{500});

  std::cout << std::endl << "---- TIMED MUTEX ----" << std::endl;
  boost::thread t1{ boost::bind(&test::Class01::set_param02, &class_01, 11, 1 ) };
  boost::thread t2{ boost::bind(&test::Class01::set_param02, &class_01, 22, 2 ) };
  t2.join();

  std::cout << std::endl << "---- SHARED MUTEX ----" << std::endl;
  std::cout << "#1: param02_ is: " << class_01.get_param02() << std::endl;
  boost::thread t3{ boost::bind(&test::Class01::get_param02, &class_01) };
  boost::thread t4{ boost::bind(&test::Class01::get_param02, &class_01) };
  t4.join();

  std::cout << std::endl << "---- NOTIFICATION ----" << std::endl;
  boost::thread wait_1{ boost::bind(&test::Class01::wait, &class_01, 1)};
  boost::thread wait_2{ boost::bind(&test::Class01::wait, &class_01, 2)};
  boost::this_thread::sleep_for(boost::chrono::milliseconds{10});
  boost::thread notify_1{ boost::bind(&test::Class01::notify, &class_01)};
  boost::this_thread::sleep_for(boost::chrono::milliseconds{10});
  boost::thread notify_2{ boost::bind(&test::Class01::notify, &class_01)};

}
