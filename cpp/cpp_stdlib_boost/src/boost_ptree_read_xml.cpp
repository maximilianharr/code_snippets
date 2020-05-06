/**
 *  @file read_config.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 23.06.2017
 *
 *  @brief Reads a text config
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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include <iostream>
#include <fstream>

// PROJECT INCLUDES

// LOCAL INCLUDES

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

/** @brief 
 *  @param 
 *  @return 
 */


// GLOBAL VARIABLES

using namespace std;

//// MAIN //////////////////////////////////////////////////////////////////////////////////////////

struct ConfigFile{
  int val01;
  double val02;
  string str01;
};

int main(int argc, char* argv[])
{

  /* Variables */
  ConfigFile config_file;
  boost::property_tree::ptree xml_read_ptree;
  boost::property_tree::ptree xml_write_ptree;
  std::string xml_read_file = "../config/config_read.xml";
  std::string xml_write_file = "../config/config_write.xml";
  std::ofstream xml_write_stream(xml_write_file); // ofstream for output needed to include xml_writer_make_settings

  /* Read XML file */
  read_xml(xml_read_file,xml_read_ptree);
  
  /* Parse config_file with xml_tree */
  config_file.val01 = xml_read_ptree.get<int>("data.val01");
  config_file.val02 = xml_read_ptree.get<double>("data.val02");
  config_file.str01 = xml_read_ptree.get<string>("data.str01");
  std::cout << "val01: " << config_file.val01 << std::endl;
  std::cout << "val02: " << config_file.val02 << std::endl;
  std::cout << "val01: " << config_file.str01 << std::endl;

  /* Put config_data in a ptree */
  xml_write_ptree.put("data.val01", 2*config_file.val01);
  xml_write_ptree.put("data.val02", 2*config_file.val02);
  xml_write_ptree.put("data.str01", config_file.str01.append("."));

  /* Write XML file */
  boost::property_tree::write_xml(xml_write_stream, xml_write_ptree, boost::property_tree::xml_writer_make_settings<std::string>('\t', 1));
  /* Also possible without xml_writer_make_settings
   * write_xml(xml_write_ptree, xml_read_ptree);*/

  return 0;
}







//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////



