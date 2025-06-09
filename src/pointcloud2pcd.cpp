#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.14/pcl/common/common.h>
#include <pcl-1.14/pcl/io/pcd_io.h>
#include <pcl-1.14/pcl/point_types.h>
#include <pcl-1.14/pcl/point_cloud.h>
#include <pcl-1.14/pcl/pcl_base.h>
#include <pcl-1.14/pcl/types.h>
#include <pcl-1.14/pcl/point_types.h>

#include <pointcloud_tools.hpp>

using std::placeholders::_1;

class PointCloud2PCD : public rclcpp::Node
{
  public:
    PointCloud2PCD() 
    : Node("pointcloud2pcd") 
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/cloud_registered", 1, std::bind(&PointCloud2PCD::pointCloudCallback, this, _1));

      this->declare_parameter("pcd_output_path", "/tmp/pointcloud.pcd");
    }
    void save_map(void)
    {
      std::string filename = this->get_parameter("pcd_output_path").as_string(); // "/tmp/pointcloud.pcd";
      if (pcl::io::savePCDFileASCII(filename, map) == 0) {
        RCLCPP_INFO(this->get_logger(), "Saved PCD file to %s", filename.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file.");
      }
      // this->create_mesh();
    }
    void create_mesh(void)
    {
      tools::greedy_projection(this->map.makeShared());
    }
    pcl::PointCloud<pcl::PointXYZ> map;
  
  private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*msg, cloud);
      map += cloud;
      save_map();

      // std::string filename = "/tmp/pointcloud.pcd";
      // if (pcl::io::savePCDFileASCII(filename, cloud)==0) {
      //   RCLCPP_INFO(this->get_logger(), "Saved PCD file to %s", filename.c_str());
      // } else {
      //   RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file.");
      // }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<PointCloud2PCD> pc2pcd = std::make_shared<PointCloud2PCD>();
  rclcpp::spin(pc2pcd);
  // User hit Ctrl+C, saving mesh and shutting down...
  rclcpp::shutdown();
  pc2pcd->create_mesh();
}
