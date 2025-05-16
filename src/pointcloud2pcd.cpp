#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
using std::placeholders::_1;

class PointCloud2PCD : public rclcpp::Node
{
  public:
    PointCloud2PCD() 
    : Node("pointcloud2pcd") 
    {
      auto subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 10, std::bind(&PointCloud2PCD::pointCloudCallback, this, _1));
    }
  
  private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*msg, cloud);

      std::string filename = "/tmp/pointcloud.pcd";
      if (pcl::io::savePCDFileASCII(filename, cloud)==0) {
        RCLCPP_INFO(this->get_logger(), "Saved PCD file to %s", filename.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file.");
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subsription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloud2PCD>());
  rclcpp::shutdown();
}
