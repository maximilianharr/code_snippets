# Project Name

About SFML (Simple and Fast Multimedia Library)
SFML is a cross-platform software development library designed to provide a simple API to various multimedia components in computers. It is written in C++ with bindings available (Java, Python, ...).

## Installation

[https://www.sfml-dev.org/tutorials/2.0/start-linux.php]
[https://github.com/SFML/SFML/wiki/Tutorial%3A-Build-your-SFML-project-with-CMake]

Install SFML
$ sudo apt-get install libsfml-dev
(SFML probably installed in /usr/include/SFML)

Compiling with cmake (not necessary to repeat this step)
Add cmake_modules folder:
$ mkdir cmake_modules

Add FindSFML.cmake to cmake_modules folder (see link)

# Detect and add SFML (in CMakeLists.txt)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})
find_package(SFML 2 REQUIRED system window graphics network audio)
target_link_libraries(${EXECUTABLE_NAME} ${SFML_LIBRARIES})

## Usage

$ cd build
$ cmake ..
$ make
$ make install

## Contributing


## History


## Credits


## License


