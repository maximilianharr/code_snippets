/**
 *  @file cpp_containers.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 01.07.2017
 *
 *  @brief Performs operations with c++ STL containers
 *
 *
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

#include <deque>
#include <queue>
#include <map>
#include <string>
#include <tuple>
#include <typeinfo>
#include <vector>

#include <algorithm>

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

template <class T>
void PrintStdContainer( T std_container){
  std::cout << "--- Printing std container (" << typeid(std_container).name() << ") ---" << std::endl;
  for( auto it = std_container.begin(); it != std_container.end(); ++it){
    std::cout << *it << std::endl;
  }
}

template <class T1, class T2>
void print_map(std::map<T1,T2> map){
  /* show content of map */
  typename std::map<T1,T2>::iterator map_iter;
  std::cout << "--- Printing map ---" << std::endl;
  for ( map_iter = map.begin(); map_iter != map.end(); ++map_iter){
    std::cout << map_iter->first << " - " << map_iter->second << std::endl;
  }
  std::cout << "--------------------" << std::endl;
}

template <class T1, class T2>
void print_multimap(std::multimap<T1,T2> map){
  /* show content of map */
  typename std::multimap<T1,T2>::iterator map_iter;
  std::cout << "--- Printing multimap ---" << std::endl;
  for ( map_iter = map.begin(); map_iter != map.end(); ++map_iter){
    std::cout << map_iter->first << " - " << map_iter->second << std::endl;
  }
  std::cout << "--------------------" << std::endl;
}

template <class T>
void print_vec( std::vector<T> vec){
  std::cout << "--- Printing vector ---" << std::endl;
  for ( int i = 0; i < vec.size(); i++){
    std::cout << vec[i] << std::endl;
  }
  std::cout << "-----------------------" << std::endl;
}

template <class T1, class T2>
void print_vec_pair( std::vector< std::pair<T1,T2> > vec){
  std::cout << "--- Printing vector ---" << std::endl;
  for ( int i = 0; i < vec.size(); i++){
    std::cout << vec[i].first << " - " << vec[i].second << std::endl;
  }
  std::cout << "-----------------------" << std::endl;
}

template <class T1, class T2, class T3>
void PrintTupleVec( std::string str, std::vector< std::tuple<T1,T2,T3> > vec_tuples){
  std::cout << std::endl << str << std::endl;
  for( auto it = vec_tuples.begin(); it != vec_tuples.end(); ++it){
    auto tuple = *it;
    std::cout << std::get<0>(tuple) << " | " << std::get<1>(tuple) << " | " << std::get<2>(tuple) << std::endl;
  }
}

std::tuple<std::string, int> getNameAge(){
  // ...
  return std::make_tuple("Bob",43);
}

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{

/* DEQUE */
  std::deque<int> my_deque;
  my_deque.push_back(0);
  my_deque.push_back(1);
  my_deque.push_back(2);
  
  for (int i = 0; i<=5; i++){
    my_deque.push_back( i+3 );
    my_deque.pop_front();
    PrintStdContainer(my_deque);
  }

/* STD::PAIR */
  std::pair<char, int> pair_0, pair_1, pair_2;
  pair_0 = std::make_pair<char, int>('a',0);
  std::cout << "pair_0" << std::endl << pair_0.first << " - " << pair_0.second << std::endl;

  /* FIND_IF */
  std::vector< std::pair<char, int> > myvec_of_pairs;
  myvec_of_pairs.push_back( std::make_pair<char, int>('a', 0) );
  myvec_of_pairs.push_back( std::make_pair<char, int>('c', 2) );
  myvec_of_pairs.push_back( std::make_pair<char, int>('d', 3) );
  myvec_of_pairs.push_back( std::make_pair<char, int>('c', 2) );
  myvec_of_pairs.push_back( std::make_pair<char, int>('e', 4) );
  myvec_of_pairs.push_back( std::make_pair<char, int>('c', 2) );
  myvec_of_pairs.push_back( std::make_pair<char, int>('f', 5) );
  myvec_of_pairs.push_back( std::make_pair<char, int>('c', 2) );
  
  print_vec_pair(myvec_of_pairs);

  /* Remove all entries that fulfill specified condition (using lambda function) */
  myvec_of_pairs.erase(
    std::remove_if(myvec_of_pairs.begin(), myvec_of_pairs.end(), 
    [&](const std::pair<char, int>& val) -> bool { return val.second == 2; }), myvec_of_pairs.end()
  );
  
  print_vec_pair(myvec_of_pairs);


/* STD::MAP */
  std::map<char, int> mymap;

  mymap.insert( std::pair<char, int>('c',2) );
  mymap.insert( std::pair<char, int>('b',2) );
  mymap.insert( std::pair<char, int>('c',3) ); // note that equal keys will be deleted!
  mymap['e'] = 4;
  mymap['d'] = 1;
  std::cout << "--- Access map by key ---" << std::endl;
  std::cout << mymap['d'] << std::endl;
  
  /* show content of map */
  print_map(mymap);

  /* Erease fields of map */
  mymap.erase('f');

  /* show content of map */
  print_map(mymap);

  /* Map with std::pair as key */
  std::map< std::pair<int,int>, char > map_pair_key;
  map_pair_key.insert( std::pair< std::pair<int,int>, char >( std::make_pair<int,int>(1,3), 'a' ) );
  map_pair_key.insert( std::pair< std::pair<int,int>, char >( std::make_pair<int,int>(2,2), 'b' ) );
  map_pair_key.insert( std::pair< std::pair<int,int>, char >( std::make_pair<int,int>(2,4), 'c' ) );
  map_pair_key.insert( std::pair< std::pair<int,int>, char >( std::make_pair<int,int>(1,2), 'd' ) );
  map_pair_key.insert( std::pair< std::pair<int,int>, char >( std::make_pair<int,int>(2,3), 'e' ) );

  std::cout << "--- MAP STD::PAIR-KEY ---" << std::endl;
  for ( auto map_iter = map_pair_key.begin(); map_iter != map_pair_key.end(); ++map_iter){
    std::cout << map_iter->first.first << " " << map_iter->first.second << " " << map_iter->second << std::endl;
  }
  std::pair<int,int> search_pair = std::make_pair<int,int>(2,3);
  auto search_pair_iter = map_pair_key.find(search_pair);
  if( search_pair_iter != map_pair_key.end()){
    std::cout << "Found : " << search_pair_iter->first.first << " " << search_pair_iter->first.second << " " 
      << search_pair_iter->second << std::endl;
  }
  std::cout << "------------------" << std::endl;

  /* Map with std::tuple as key */
  std::map< std::tuple<int,int,std::string>, char > map_tuple_key;
  map_tuple_key.insert( std::pair< std::tuple<int,int,std::string>, char >( std::make_tuple(2,3,"b"), 'a' ) );
  map_tuple_key.insert( std::pair< std::tuple<int,int,std::string>, char >( std::make_tuple(1,3,"c"), 'b' ) );
  map_tuple_key.insert( std::pair< std::tuple<int,int,std::string>, char >( std::make_tuple(1,3,"a"), 'c' ) );
  map_tuple_key.insert( std::pair< std::tuple<int,int,std::string>, char >( std::make_tuple(1,3,"ab"), 'c' ) );
  map_tuple_key.insert( std::pair< std::tuple<int,int,std::string>, char >( std::make_tuple(2,3,"a"), 'd' ) );
  map_tuple_key.insert( std::pair< std::tuple<int,int,std::string>, char >( std::make_tuple(1,4,"a"), 'e' ) );

  std::cout << "--- MAP STD::TUPLE-KEY ---" << std::endl;
  for ( auto map_iter = map_tuple_key.begin(); map_iter != map_tuple_key.end(); ++map_iter){
    std::cout << std::get<0>(map_iter->first) << " " << std::get<1>(map_iter->first) << " " 
      << std::get<2>(map_iter->first) << " " << map_iter->second << std::endl;
  }
  std::tuple<int,int,std::string> search_tuple = std::make_tuple<int,int,std::string>(2,3,"a");
  auto search_tuple_iter = map_tuple_key.find(search_tuple);
  if( search_tuple_iter != map_tuple_key.end()){
    std::cout << "Found : " << std::get<0>(search_tuple_iter->first) << " "  << std::get<1>(search_tuple_iter->first) << " "
       << std::get<2>(search_tuple_iter->first) << " "  << search_tuple_iter->second << std::endl;
  }
  unsigned long long int sensor_0_id = 1;
  std::tuple<unsigned long long int,unsigned long long int,std::string> residual_key = std::make_tuple(sensor_0_id, sensor_0_id,"rtk");
  std::cout << "------------------" << std::endl;
    
/* STD::VECTOR */
  std::vector<int> myvec;  
  
  myvec.push_back(2);
  myvec.push_back(4);
  myvec.push_back(6);

  print_vec(myvec);
  for ( int i = 0; i < myvec.size(); i++){
    if( myvec[i] == 4){
      myvec.erase(myvec.begin()+i);
    }
  }
  print_vec(myvec);

/* MULTIMAP (keys can occur multiple times */
  std::multimap<char,int> mymultimap;
  mymultimap.insert( std::pair<char, int>('c',2) );
  mymultimap.insert( std::pair<char, int>('b',2) );
  mymultimap.insert( std::pair<char, int>('c',3) ); // note that equal keys will NOT be deleted!
  mymultimap.insert( std::pair<char, int>('a',2) );
  print_multimap(mymultimap);

  /* Erase key */
  mymultimap.erase('c');
  print_multimap(mymultimap);

  /* TUPLE (introduced in C++11) 
   * https://www.youtube.com/watch?v=33Y_5gOyi3Y&index=19&list=PL5jc9xFGsL8FWtnZBeTqZBbniyw0uHyaH */
  std::vector< std::tuple<int, int, char> > vec_tuples;
  for( int i = 0; i<3; i++){
    vec_tuples.push_back( std::make_tuple(i, 2*i, 'A') );}
  PrintTupleVec<int, int, char>("vec_tuples: ", vec_tuples);

  /* Change value of tuples by accessing element directly (compare to std_vector.at(i) )*/
  std::get<2>(vec_tuples.at(0)) = 'X';

  /* Change value of tuples by getting reference */
  int& val = std::get<0>(vec_tuples.at(0)); // get reference to tuple element
  val = 2;
  // std::get<val>(vector_of_tuples.at(0)) = 'X'; // val MUST BE compile time constant !
  PrintTupleVec<int, int, char>("vec_tuples (changed first elements): ", vec_tuples);

  /* Store elements of tuple with std::tie in variables */
  int val0;
  char val2;
  std::tie(val0, std::ignore, val2) = vec_tuples.at(2);
  std::cout << "val0: " << val0 << " | val2: " << val2 << std::endl;

  /* Tuple can store references (no stdlib containters can do that) */
  std::string st = "hello world!";
  std::tuple<std::string&> tuple_st(st); // tuple_st is reference to string st
  std::get<0>(tuple_st) = "hello c++!";
  std::cout << std::endl << std::get<0>(tuple_st) << std::endl;

  /* combining tuples */
  auto tuple_cat = std::tuple_cat(vec_tuples.at(0), tuple_st); // <int, int, char, string&>

  /* Get tuple information */
  std::cout << "Number of tuple_cat elements: " << std::tuple_size<decltype(tuple_cat)>::value << std::endl; // 4
  std::tuple_element<1, decltype(tuple_cat)>::type d; // don't know how to use this

// WHEN TO USE TUPLE
  /* Why prefer tuple instead of structure ? *
   * Structure makes code more readable
   */

  /* Tuple as one-time only structure to transfer data */
  std::string name;
  int age;
  tie(name,age) = getNameAge(); // That is a nice feature. Returning multiple values without the need to create a specific structur
  std::cout << std::endl << "Name: " << name << " | Age: " << age << std::endl;

  /* Use tuples for Comparison */
  std::tuple<int, int, int> time1, time2;
  time1 = std::make_tuple(11,59,58);
  time2 = std::make_tuple(11,59,59);
  if( time1 < time2){
    std::cout << std::endl << "time 1 < time2" << std::endl << std::endl; }

  /* Multi index map 
   * If used again and again it might be a good idea to use a structure instead of tuple > Readability! */
  std::map< std::tuple<int, char, float>, std::string > tuplemap;
  tuplemap[ std::make_tuple(1, 'a', 1.0) ] = "Faith can move mountains";
  tuplemap[ std::make_tuple(2, 'b', 2.0) ] = "Action speaks louder than words";
  tuplemap[ std::make_tuple(3, 'c', 3.0) ] = "Picture paints a thousand words";
  tuplemap[ std::make_tuple(4, 'd', 4.0) ] = "Every cloud has a silver lining";

  /* Shift values between variables */
  int a = 0;
  int b = 1;
  int c = 2;
  std::cout << "a: " << a << " |b: " << b << " |c: " << c << std::endl;
  std::tie(c, a, b) = std::make_tuple(a, b, c);
  std::cout << "a: " << a << " |b: " << b << " |c: " << c << std::endl;

// Additional ( Working with map of tuples )

  /* Search an element and remove it */
  auto it = tuplemap.find( std::make_tuple(2, 'b', 2.0) );
  if (it != tuplemap.end()){ 
    std::cout << "Removing : " << it->second << std::endl << std::endl;
    tuplemap.erase(it);
  }

  /* Search for elements in map */
  for ( auto it = tuplemap.begin(); it != tuplemap.end(); ++it ){
    auto map_element = *it;
    if( map_element.first == std::make_tuple(1, 'a', 1.0) ){
      std::cout << "Possitive: " << map_element.second << std::endl; }
    else if( std::get<0>(map_element.first) > 1 && std::get<2>(map_element.first) < 3.9 ){
      std::cout << "In Range:  " << map_element.second << std::endl;
    }
    else{
      std::cout << "Negative:  " << map_element.second << std::endl; }
  }

  return 0;
}


//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


