#include<vector>
#include<iostream>
#include<utility>

#include <pcl/visualization/pcl_plotter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/feature.h>
#include <pcl/features/impl/feature.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/impl/shot_omp.hpp>

using namespace std;

int
main ()
{
  //defining a plotter
  pcl::visualization::PCLPlotter * plotter = new pcl::visualization::PCLPlotter ();

  //defining the polynomial function, y = x^2. Index of x^2 is 1, rest is 0
  vector<double> func1 (3,0);
  func1[2] = 1;

  //adding the polynomial func1 to the plotter with [-10, 10] as the range in X axis and "y = x^2" as title
  plotter->addPlotData (func1, -10, 10, "y = x^2");

  //display the plot, DONE!
  plotter->plot ();

  return 0;
}

