# CMake-Tutorial https://cmake.org/cmake-tutorial/

# CONFIG
  # Check if minimum cmake-version is installed
  cmake_minimum_required(VERSION 2.6)

  # Set name of project and store in PROJECT_NAME
  # Additionally sets PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR
  project(effective_modern_cpp)

# Compile using c++14 standard
  set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# Include SFML-library (for interaction with computer multimedia) 
  set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})
  find_package(SFML 2 REQUIRED system window graphics network audio)

# OUTPUT
  # Set colors for output [http://stackoverflow.com/questions/18968979/how-to-get-colorized-output-with-cmake]
  string(ASCII 27 Esc)
  set(ColorReset "${Esc}[m")
  set(Info "${Esc}[32m") # Green

  # Print messages on terminal
  message(${Info} "Project-name: " ${PROJECT_NAME} ${ColorReset})
  message("Building make-files ...")

# PASS VARIABLES TO EXECUTEABLE
  # Add version number in CMakeLists that is transfered to executeable
  set(effective_modern_cpp_VERSION_MAJOR 1)
  set(effective_modern_cpp_VERSION_MINOR 0)
  # Select file (*.h.in) with config values. Generate file (*.h) in binary dir.
  configure_file(
    ${PROJECT_SOURCE_DIR}/include/effective_modern_cpp_config.h.in
    ${PROJECT_BINARY_DIR}/effective_modern_cpp_config.h
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
  file(GLOB SOURCES src/*.cpp)

# ADD ALL EXECUTEABLES
  # find all *.cpp files
  file(GLOB SRC_SOURCES src/*.cpp)
  # iterate through all cpp-files
  foreach(src_file ${SRC_SOURCES})
    
    # Remove cpp extension and filepath
    string(REPLACE ".cpp" "" exec_name ${src_file})
    string(REPLACE "${PROJECT_SOURCE_DIR}/src/" "" exec_name ${exec_name})
    message(${INFO} "Found executables: " ${exec_name} ${ColorReset})
    
    # Add executables and link libraries
    add_executable(${exec_name} ${src_file})
   
    # Link executables to sfml library 
    target_link_libraries(${exec_name} ${SFML_LIBRARIES})

  endforeach()

# INSTALL (move executable files to specified folder)
  foreach(src_file ${SRC_SOURCES})
    # Remove cpp extension and filepath
    string(REPLACE ".cpp" "" exec_name ${src_file})
    string(REPLACE "${PROJECT_SOURCE_DIR}/src/" "" exec_name ${exec_name})
    message(${INFO} "Found executeables: " ${exec_name} ${ColorReset})
    install(TARGETS ${exec_name} DESTINATION ${PROJECT_SOURCE_DIR}/bin)
  endforeach()

# TESTING (Check if application is running correctly)
include(CTest)
  # Add test called 'test01' that runs 'chapter01' on console ...
  add_test(test01 chapter01)
  # ... and check if output is as described
  set_tests_properties(test01 PROPERTIES PASS_REGULAR_EXPRESSION "Programm finished successfully.")



