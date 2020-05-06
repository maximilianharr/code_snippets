/**
 *  @file read_json.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 08.12.2017
 *
 *  @brief	Read parameter from json-file
 *
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
 *
 *
 *  @bug
 *
 *  @todo
 *
 */

#ifdef _MSC_VER
#include <boost/config/compiler/visualc.hpp>
#endif
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <cassert>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>

int main()
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_json("/lhome/harrmax/workspace/daimler_harr_workspace/cpp/cpp_read_systemparams/cfg/data.json", pt);
  std::string msg = pt.get<std::string>("covariance.topview.lon");
  std::cout << msg << std::endl;

  if( pt.get<bool>("dump.bool") ){
    std::cout << "dump" << std::endl;
  }
}
