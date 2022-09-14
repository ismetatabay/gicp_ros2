#include <iostream>
#include <thread>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

// ROS2 Node 
class GICP : public rclcpp::Node
{
public:
  GICP()
  : Node("gicp_node"), count_(0)
  {
    this->declare_parameter<std::string>("pcd1_filename", "");
    this->declare_parameter<std::string>("pcd2_filename", "");
    publisher_transformed = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transformed_pointcloud", 10);
    publisher_target = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_pointcloud", 10);
    publisher_input = this->create_publisher<sensor_msgs::msg::PointCloud2>("/input_pointcloud", 10);
    GICP::gicp_();
    timer_ = this->create_wall_timer(
      500ms, std::bind(&GICP::pcd_pub, this));
  }

private:
  // Generalized Iterative Closest Point
  int gicp_()
  {
    this->get_parameter("pcd1_filename", tgt_filename);
    this->get_parameter("pcd2_filename", src_filename);
    //pcl::PointCloud<pcl::PointXYZ> output_cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (tgt_filename, *tgt) == -1)
    {
      RCLCPP_INFO(this->get_logger(), "Couldn't read file %s \n", tgt_filename.c_str());
      return (-1);
    }
    std::cout << "Loaded " << tgt->size () << " data points from tgt: " << tgt_filename << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (src_filename, *src) == -1)
    {
      RCLCPP_INFO(this->get_logger(), "Couldn't read file %s \n", src_filename.c_str());
      return (-1);
    }
    //std::cout << "Loaded " << src->size () << " data points from src: "<<  src_filename << std::endl;
    RCLCPP_INFO(this->get_logger(), "Loaded %d data points from src:  %s \n", src->size (), src_filename.c_str());

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
    // gicp          
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource (filtered_cloud);
    gicp.setInputTarget (tgt);
    gicp.setMaxCorrespondenceDistance (30);
    gicp.setMaximumIterations (100);
    gicp.setTransformationEpsilon (1e-8);
    gicp.setEuclideanFitnessEpsilon (1e-6);
    gicp.setRANSACIterations(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align (*output_cloud);

    RCLCPP_INFO(this->get_logger(),"Normal Distributions Transform has converged: %d score: %d ", 
                  gicp.hasConverged(), gicp.getFitnessScore ());
    // transform and save pcl
    pcl::transformPointCloud (*src, *output_cloud, gicp.getFinalTransformation ());
    pcl::io::savePCDFileASCII ("src/gicp_ros2/data/transformed.pcd",*output_cloud);

    pcl::copyPointCloud(*src,input_cloud);
    pcl::copyPointCloud(*tgt,target_cloud);
    pcl::copyPointCloud(*output_cloud,transformed_cloud);

    return (0);
  }
  void pcd_publisher()
  { // coloring
    auto pc2_message = sensor_msgs::msg::PointCloud2();
    for(auto &p: transformed_cloud.points){
      p.r = 255;
      p.g = 0;
      p.b = 0;
    }
    for(auto &p: target_cloud.points){
      p.r = 0;
      p.g = 255;
      p.b = 0;
    }
    for(auto &p: input_cloud.points){
      p.r = 255;
      p.g = 255;
      p.b = 255;
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing PointClouds");
    
    pcl::toROSMsg(transformed_cloud, pc2_message);
    pc2_message.header.frame_id = "map";
    pc2_message.header.stamp = now();
    publisher_transformed->publish(pc2_message);

    pcl::toROSMsg(target_cloud, pc2_message);
    pc2_message.header.frame_id = "map";
    pc2_message.header.stamp = now();
    publisher_target->publish(pc2_message);

    pcl::toROSMsg(input_cloud, pc2_message);
    pc2_message.header.frame_id = "map";
    pc2_message.header.stamp = now();
    publisher_input->publish(pc2_message);

  }
  std::string tgt_filename, src_filename;
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> target_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_transformed;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_target;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_input;
  size_t count_;
};
int
main (int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GICP>());
  rclcpp::shutdown();
  return (0);
}
