/**
 *  @file prot_write.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 12.10.2016
 *
 *  @brief Writes protobuf binary file
 *         See: https://developers.google.com/protocol-buffers/docs/cppopel#reading-a-message
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
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  /* Generate LocData */
  LocData loc_data;

  loc_data.set_lat(49.98989);
  loc_data.set_lon(8.405400);
  loc_data.set_valid(1);

  /* Write the new address book back to disk. */
  fstream output("loc_data.txt", ios::out | ios::trunc | ios::binary);
  if (!loc_data.SerializeToOstream(&output)) {
    cerr << "Failed to write address book." << endl;
    return -1;
  }

  // Optional:  Delete all global objects allocated by libprotobuf.
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}

//// FUNCTION DEFINITIONS //////////////////////////////////////////////////////////////////////////


