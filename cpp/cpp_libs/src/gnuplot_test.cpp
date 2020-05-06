/*
  sinus.c
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <map>
#include <vector>
#include <cmath>

#include "gnuplot-iostream.h"

int main()
{
  Gnuplot gp;
 // gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
  gp << "plot sin(x)\n";

  std::cout << "Press enter to exit." << std::endl;
  std::cin.get();


//  system("echo 'set xrange [-4:4]' | gnuplot");
//  system("echo 'plot sin(x); pause 5' | gnuplot");
 
  return(0);
}

