# CMake-Tutorial https://cmake.org/cmake-tutorial/

# CONFIG
  # Check if minimum cmake-version is installed
  cmake_minimum_required(VERSION 2.6)

  # Set name of project and store in PROJECT_NAME
  # Additionally sets PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR
  project(cpp)

# Add local cmake-modules (You can define in *.cmake where cmake shall search your library, eg. MrtPCL)
# That way you can assure that each computer links it's programm with the same library version/
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

  # Compile using c++14 standard
  set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# OUTPUT
  # Set colors for output [http://stackoverflow.com/questions/18968979/how-to-get-colorized-output-with-cmake]
  string(ASCII 27 Esc)
  set(ColorReset "${Esc}[m")
  set(Info "${Esc}[32m") # Green

  # Print messages on terminal
  message(${Info} "Project-name: " ${PROJECT_NAME} ${ColorReset})
  message(${Info} "Building make-files ..." ${ColorReset})

# PASS VARIABLES TO EXECUTEABLE
  # Add version number in CMakeLists that is transfered to executeable
  set(cpp_VERSION_MAJOR 1)
  set(cpp_VERSION_MINOR 0)
  # Select file (*.h.in) with config values. Generate file (*.h) in binary dir.
  configure_file(
    ${PROJECT_SOURCE_DIR}/include/project_config.h.in
    ${PROJECT_BINARY_DIR}/project_config.h
  )
  # Add binary dir where *.h has been created. Include this file in *.cpp-files!
  include_directories(
    ${PROJECT_BINARY_DIR}
  )

# INCLUDE
  # Directories
  include_directories(
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/include
  )
  # All *.cpp-files in specified folder
  file(GLOB SOURCES include/*.cpp)

# ADD ALL EXECUTEABLES
  # find all *.cpp files
  file(GLOB SRC_SOURCES src/*.cpp)
  # iterate through all cpp-files
  foreach(src_file ${SRC_SOURCES})
    # Remove cpp extension and filepath
    string(REPLACE ".cpp" "" exec_name ${src_file})
    string(REPLACE "${PROJECT_SOURCE_DIR}/src/" "" exec_name ${exec_name})
    message(${Info} "Found executeables: " ${exec_name} ${ColorReset})
    # Add executables and link libraries
    add_executable(${exec_name} ${src_file})
    target_link_libraries(${exec_name} )
    # Put executables in bin folder (installing) 
    install(TARGETS ${exec_name} DESTINATION ${PROJECT_SOURCE_DIR}/bin)
  endforeach()


