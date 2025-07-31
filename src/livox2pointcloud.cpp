#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <livox_interfaces/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using PointField = sensor_msgs::msg::PointField;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

using std::placeholders::_1;

class LivoxConverter : public rclcpp::Node
{
    public:
        LivoxConverter() 
        : Node("livox2pointcloud")
        {
            points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/points", 1);

            // auto livox_sub = this->create_subscription<livox_interfaces::msg::CustomMsg>("/livox/lidar", rclcpp::SensorDataQoS(), &LivoxConverter::convert);
            livox_sub = this->create_subscription<livox_interfaces::msg::CustomMsg>("/livox/lidar", 1, std::bind(&LivoxConverter::convert, this, _1));
        };

        void convert(const livox_interfaces::msg::CustomMsg::ConstSharedPtr livox_msg) 
        {
            pcl::PointCloud<pcl::PointXYZI> converted_cloud;
            converted_cloud.height = 1;
            converted_cloud.width = livox_msg->point_num;
            converted_cloud.is_dense = false;
            converted_cloud.resize(livox_msg->point_num);

            int i = 0;
            for (auto& points: converted_cloud)
            {
                points.x = livox_msg->points[i].x;
                points.y = livox_msg->points[i].y;
                points.z = livox_msg->points[i].z;
                points.intensity = livox_msg->points[i++].reflectivity;
            }
            converted_cloud.header = pcl_conversions::toPCL(livox_msg->header);
            converted_cloud.header.frame_id = "body";

            sensor_msgs::msg::PointCloud2 ros_cloud;
            pcl::toROSMsg(converted_cloud, ros_cloud);
            points_pub->publish(ros_cloud);

        };
    private:
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
        rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr livox_sub;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<LivoxConverter> livox2pointcloud = std::make_shared<LivoxConverter>();

  rclcpp::spin(livox2pointcloud);
  rclcpp::shutdown();

}
