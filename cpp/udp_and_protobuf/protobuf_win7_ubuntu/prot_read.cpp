/**
 *  @file prot_read.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 12.10.2016
 *
 *  @brief Reads protobuf binary file and prints data
 *         See: https://developers.google.com/protocol-buffers/docs/cppopel#reading-a-message
 *
 *	    Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
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
#include <iostream>
#include <fstream>
#include <string>

// PROJECT INCLUDES

// LOCAL INCLUDES
#include "loc_data.pb.h"

// FORWARD REFERENCES

// FUNCTION PROTOTYPES

// GLOBAL VARIABLES

using namespace std;
//// MAIN //////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {

  /* Verify that the version of the library that we linked against is
   * compatible with the version of the headers we compiled against.*/
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  /* Generate LocData */
  LocData loc_data;
  {
    /* Read the existing address book. */
    fstream input("loc_data.txt", ios::in | ios::binary);
    if (!loc_data.ParseFromIstream(&input)) {
      cerr << "Failed to parse loc_data." << endl;
      return -1;
    }
  }
  std::cout << "Localization data:\n" ;
  std::cout << "Lat:   " << loc_data.lat() << "\n";
  std::cout << "Lon:   " << loc_data.lon() << "\n";
  std::cout << "Valid: " << loc_data.valid() << "\n";


  // Optional:  Delete all global objects allocated by libprotobuf.
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}

//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////
