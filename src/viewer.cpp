#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int
main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/gicp_ros2/data/capture0001.pcd", *tgt) == -1)
  {
    PCL_ERROR ("Couldn't read file capture0001.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << tgt->size () << " data points from tgt: capture0001.pcd" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/gicp_ros2/data/capture0002.pcd", *src) == -1)
  {
    PCL_ERROR ("Couldn't read file capture0002.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << src->size () << " data points from src: capture0002.pcd" << std::endl;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*src, *src, indices);
  pcl::removeNaNFromPointCloud(*tgt, *tgt, indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (src);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from capture0002.pcd" << std::endl;

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource (filtered_cloud);
  gicp.setInputTarget (tgt);
  //gicp.setMaximumIterations (50);
  //gicp.setTransformationEpsilon (1e-8);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  gicp.align (*output_cloud);

  std::cout << "Normal Distributions Transform has converged:" << gicp.hasConverged ()
            << " score: " << gicp.getFitnessScore () << std::endl;

  pcl::transformPointCloud (*src, *output_cloud, gicp.getFinalTransformation ());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("src/gicp_ros2/data/transformed.pcd",*output_cloud);

  // // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);
  
  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  input_color (src, 0, 0, 255);
  viewer_final->addPointCloud<pcl::PointXYZ> (src, input_color, "input cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (tgt, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (tgt, target_color, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "transformed input cloud");


  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);
}
