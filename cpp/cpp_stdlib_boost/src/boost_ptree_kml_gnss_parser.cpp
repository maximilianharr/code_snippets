/**
 *  @file kml_gnss_parser.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 05.01.2016
 *
 *  @brief This is a sample program how to use ptree of boost for xml editing and kml creation
 *         Has been included in ROS. in kml_converter.cpp
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
/*
 * Property tree:
 * http://www.boost.org/doc/libs/1_41_0/doc/html/property_tree/reference.html#header.boost.property_tree.xml_parser_hpp
 *
 * Plot in:
 * http://display-kml.appspot.com/
*/
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>

int init_kml_file(std::string directory, std::string filename)
{
  std::string full_path;
  std::ofstream kml_file;

  /* Open KML file */
  full_path = directory;
  full_path.append(filename);
  kml_file.open(full_path);

  if(kml_file.is_open())
  {
    /* Insert KML base layout */
    kml_file
      << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
      << "<kml xmlns=\"http://earth.google.com/kml/2.2\">"
      <<   "<Document>"
      <<     "<name>3_point_Chur.kml</name>"
      <<     "<StyleMap id=\"my_Style\">"
      <<       "<Pair>"
      <<         "<key>normal</key>"
      <<         "<styleUrl>#my_Style_id</styleUrl>"
      <<       "</Pair>"
      <<     "</StyleMap>"
      <<     "<Style id=\"my_Style_id\">"
      <<       "<LineStyle>"
      <<         "<color>ff00ffff</color>"
      <<         "<width>10</width>"
      <<       "</LineStyle>"
      <<     "</Style>"
      <<     "<Placemark>"
      <<       "<name>3_point_Chur</name>"
      <<       "<styleUrl>#my_Style</styleUrl>"
      <<       "<LineString>"
      <<         "<coordinates>"
      <<         "</coordinates>"
      <<       "</LineString>"
      <<     "</Placemark>"
      <<   "</Document>"
      << "</kml>";
  }
  else
  {
    return 1;
  }

  /* Close KML file */
  kml_file.close();
  return 0;
}

float gnss_deg2dec(float gnss_deg)
{
  float gnss_dec;

  /* Adopt degrees */
  gnss_dec = floor(gnss_deg/100);
  /* Degrees can now be subtracted */
  gnss_deg = gnss_deg - gnss_dec*100;
  /* Add Minutes */
  gnss_dec += (gnss_deg)/60;

  return gnss_dec;
}

int main()
{
  using boost::property_tree::ptree;

  /* variables */
  ptree kml_pt;
  std::string kml_directory;
  std::string kml_filename;
  std::string gps_waypoints = "";
  std::string kml_full_path;
  std::stringstream ss;
  time_t t;
  struct tm * now;

  /* Get the current time and append on filename */
  t = time(0); /* see ctime-header */
  now = localtime( & t ); /* http://www.cplusplus.com/reference/ctime/tm/ */
  ss << (now->tm_year + 1900)
            << std::setw(2) << std::setfill('0') << (now->tm_mon + 1)
            << std::setw(2) << std::setfill('0') << now->tm_mday << '_'
            << std::setw(2) << std::setfill('0') << now->tm_hour
            << std::setw(2) << std::setfill('0') << now->tm_min
            << std::setw(2) << std::setfill('0') << now->tm_sec;

  /* Get filename and path */
  kml_directory = "./";
  kml_filename = ss.str();
  kml_filename.append("_gnss_waypoints.kml");

  /* Prepare KML file */
  init_kml_file(kml_directory,kml_filename);

  /* Read KML file */
  kml_full_path = kml_directory;
  kml_full_path.append(kml_filename);
  read_xml(kml_full_path,kml_pt);

  /* Access data and print */
  gps_waypoints.append("9.5310683,46.848633,614.7\n");
  gps_waypoints.append("9.5310684,46.848742,617.8\n");
  gps_waypoints.append("9.5310073,46.848861,613.8\n");
  kml_pt.put("kml.Document.Placemark.LineString.coordinates",gps_waypoints);
  write_xml(kml_full_path, kml_pt);

  std::cout << std::fixed << std::setprecision(8) << gnss_deg2dec(4650.9180) << "\n";

  return 0;
}
