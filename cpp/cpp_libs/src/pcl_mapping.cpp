/**
 *  @file pcl_mapping.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 04.12.2017
 *
 *  @brief Point cloud library [http://www.pointclouds.org/]
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


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/feature.h>
#include <pcl/features/impl/feature.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp> 
#include <pcl/features/impl/shot_omp.hpp>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
      cloud_in->points[i].z << std::endl;

  /* Map data of pointcloud to std::vector of Vector3d */
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_of_points;
  for(auto it = cloud_in->begin(); it < cloud_in->end(); ++it){
    vector_of_points.push_back( it->getVector3fMap().cast<double>() );
  }

  std::cout << " Vector: " << std::endl;
  for(auto it = vector_of_points.begin(); it < vector_of_points.end(); ++it){
    std::cout << *it << std::endl;
  }


 return (0);
}
