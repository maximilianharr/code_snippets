/**
 *  @file thread_local_storage.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 30.11.2017
 *
 *  @brief Thread Local Storage (TLS) is a dedicated storage area that can only be accessed by one thread.
 *         See: https://theboostcpplibraries.com/boost.thread-thread-local-storage
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
#include <boost/thread/scoped_thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>

namespace test {

class Class01{
  public:
    /* mutex is to prevent other threads from taking ownership while a particular thread owns the mutex */
    boost::mutex mutex01_;
    static bool init_static_;
    /* Since TLS variables are global per thread, not global per process,
     * using tls in one thread does not change the variable in any other thread. */
    static boost::thread_specific_ptr<bool> init_tls_;

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
    /** @brief Initialization (Static)
     */
    void init_static(int id)
    {
      boost::lock_guard<boost::mutex> lock{mutex01_};
      if (!init_static_)
      {
        init_static_ = true;
        std::cout << "Initialization complete (static): " << id  << std::endl;
      }
    }

    /** @brief Initialization (TLS)
     */
    void init_tls(int id)
    {
      if( !init_tls_.get() ){ // Check wheater an adress has already been stored in init_tls_ with get() member function
        /* variable of type bool is dynamically allocated and its address, returned by new, is stored in init_tls_ */
        init_tls_.reset(new bool{true});
        boost::lock_guard<boost::mutex> lock{mutex01_};
        std::cout << "Initialization complete (tls): " << id  << std::endl;
        return;
      }
    }

    /** @brief Init and then do workload (Static)
     */
    void workload_static(int id)
    {
      init_static(id);
      init_static(id);
      std::cout << "Working on something (static): " << id << std::endl;
    }

    /** @brief Init and then do workload (TLS)
     */
    void workload_tls(int id)
    {
      init_tls(id);
      init_tls(id);
      std::cout << "Working on something (tls): " << id  << std::endl;
    }

  private:
    int param01_, param02_;
};
bool Class01::init_static_;
boost::thread_specific_ptr<bool> Class01::init_tls_;


} // end of namespace test

int main()
{
  /* Create a Class01 object */
  std::cout << std::endl << "------------------" << std::endl;
  test::Class01 class_01(10,0);

  std::cout << std::endl << "---- Synchronizing multiple threads with static variables ----" << std::endl;
  /* Open 3 threads
   * Initialization only done once because static member init_static_ */
  boost::thread tstatic[3];
  for (int i = 0; i < 3; ++i){
    tstatic[i] = boost::thread{ boost::bind(&test::Class01::workload_static, &class_01, i) };
  }
  /* Wait until all 3 threads have finished */
  for (int i = 0; i < 3; ++i){
    tstatic[i].join();
  }

  std::cout << std::endl << "---- Synchronizing multiple threads with TLS ----" << std::endl;
  /* Open 3 threads
   * Initialization only done once because TLS member init_tls_ */
  boost::thread ttls[3];
  for (int i = 0; i < 3; ++i){
    ttls[i] = boost::thread{ boost::bind(&test::Class01::workload_tls, &class_01, i) };
  }
  /* Wait until all 3 threads have finished */
  for (int i = 0; i < 3; ++i){
    ttls[i].join();
  }
}
