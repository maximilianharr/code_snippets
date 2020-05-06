COMPILING*:
  cd build
  [ go to build folder. Here all CMake files shall be created ]
  cmake ../
  [ Build Makefile from CMakeLists.txt ]
  make
  [ Build executeable using Makefile ]
  ctest
  [ Check if all tests passed ]
  make install
  [ Put executeables to specified folder ]

* Note: Run "cmake ../" twice if project is on USB-Harddrive (don't know why)


FOLDER STRUCTURE: [http://hiltmon.com/blog/2013/07/03/a-simple-c-plus-plus-project-structure/]

bin:
The output executables go here, both for the app and for any tests and spikes.

build: 
This folder contains all object files, and is removed on a clean.

doc: 
Any notes, like assembly notes and configuration files, are here.

include: 
All project header files. All necessary third-party header files that do not exist under /usr/local/include are also placed here.

lib: 
Any libs that get compiled by the project, third party or any needed in development. Prior to deployment, third party libraries get moved to /usr/local/lib where they belong, leaving the project clean enough to compile on our Linux deployment servers. Use this to test different library versions than the standard.

spike: 
Smaller classes or files to test technologies or ideas, and keep them around for future reference. They go here, where they do not dilute the real application’s files, but can still be found later.

src: 
The application and only the application’s source files.

test: 
All test code files.

