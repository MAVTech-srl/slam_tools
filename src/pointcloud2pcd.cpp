#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// #include <std_msgs/msg/string.hpp>
// #include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.14/pcl/common/common.h>
#include <pcl-1.14/pcl/io/pcd_io.h>
#include <pcl-1.14/pcl/point_types.h>
#include <pcl-1.14/pcl/point_cloud.h>
#include <pcl-1.14/pcl/pcl_base.h>
#include <pcl-1.14/pcl/types.h>
#include <pcl-1.14/pcl/point_types.h>
#include "rclcpp/wait_for_message.hpp"
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <pointcloud_tools.hpp> 

#include <cassert>
#include <cmath>   // for HUGE_VAL
#include <iomanip> // for std::setprecision()
#include "proj.h"    

#include <iostream>
#include <fstream>
#include <memory>
using std::placeholders::_1;

using namespace sensor_msgs::msg;
using namespace nav_msgs::msg;

class PointCloud2PCD : public rclcpp::Node
{
  public:
    PointCloud2PCD() 
    : Node("pointcloud2pcd"),
    ctx(PJ_DEFAULT_CTX) 
    {
      // Callbacks for gps and odometry
      auto gpsCallback =
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void
        {
            RCLCPP_INFO(this->get_logger(), "GPS");
        };
      auto odometryCallback =
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void
        {
            RCLCPP_INFO(this->get_logger(), "Odometry");
        };

      // auto callback =
      //       // [this](const std_msgs::msg::String::SharedPtr msg) -> void
      //       [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) ->void
      //   {
      //       RCLCPP_INFO(this->get_logger(), "I heard diagnostics");
      //   };
      // sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10, callback);

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      // Initialize subscribers
      subs_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/cloud_registered", 1, std::bind(&PointCloud2PCD::pointCloudCallback, this, _1));
      subs_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/raw/fix", qos, gpsCallback);
      subs_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/mavros/local_position/odom", qos, odometryCallback);

      
      this->declare_parameter("pcd_output_path", "/tmp/pointcloud.pcd");
      std::string filename = this->get_parameter("pcd_output_path").as_string();
      
      pcl::PointCloud<pcl::PointXYZI> dummy_pointcloud;
      this->dummy_pointcloud.height = 0;
      this->dummy_pointcloud.width = 0;
      this->dummy_pointcloud.is_dense = false;
      this->dummy_pointcloud.resize(0);
      
      std::ofstream outfile;
      outfile.open("pointcloud.pcd", std::ios::app);

      projector = proj_create_crs_to_crs(ctx, 
                                          "EPSG:4326",  // WGS84 as provided by PX4 message
                                          "+proj=utm +zone=32 +datum=WGS84", 
                                          NULL);

      if (0 == projector) 
      {
        fprintf(stderr, "Failed to create transformation object.\n");
        return;
      }
      /* This will ensure that the order of coordinates for the input CRS */
      /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
      /* longitude */
      norm = proj_normalize_for_visualization(ctx, projector);
      if (0 == norm) 
      {
        fprintf(stderr, "Failed to normalize transformation object.\n");
        return;
      }
      proj_destroy(projector);
      projector = norm;
    }
    std::ofstream outfile;
  
  private:
    void pointCloudCallback(const PointCloud2::SharedPtr msg)
    {
      if ( point_count == 0 )
      {
        NavSatFix::SharedPtr msg_gps;
        Odometry::SharedPtr msg_odometry;

      // // wait for test message
      //   node_ = std::make_shared<rclcpp::Node>("wait_for_msg_node");
      //   std::thread(
      //       [&]()
      //       {
      //           while (rclcpp::ok())
      //           {
      //               diagnostics_msgs::msg::DiagnosticArray out;
      //               auto ret = rclcpp::wait_for_message(out, node_, "/diagnostics");
      //               if (ret)
      //                   RCLCPP_INFO(node_->get_logger(), "I heard diagnostics);
      //           }
      //       }).detach();
    
        //wait for msg_gps
        node_gps_ = std::make_shared<rclcpp::Node>("wait_for_msg_gps_node");
        std::thread(
          [&]()
          {
            while (rclcpp::ok()){
              auto ret = rclcpp::wait_for_message(msg_gps, node_gps_, "/mavros/global_position/raw/fix");
              if (ret)
                RCLCPP_INFO(node_gps_->get_logger(), "gps_msg received");
            }
          }
        ).detach();

        //wait for msg_odom
        node_odom_ = std::make_shared<rclcpp::Node>("wait_for_msg_odom_node");
        std::thread(
          [&](){
            while(rclcpp::ok()){
              auto ret = rclcpp::wait_for_message(msg_odometry, node_odom_, "/mavros/local_position/odom");
              if (ret)
                RCLCPP_INFO(node_odom_->get_logger(), "odom_msg received");
            }
          }
        ).detach();

        // Perform the coordinate transformation.
        PJ_COORD coord_in;
        coord_in.lpzt.z = msg_gps->altitude;            // z ordinate. unused (altitude already in meters)
        coord_in.lpzt.t = HUGE_VAL;       // time ordinate. unused
        coord_in.lp.lam = msg_gps->longitude;   // longitude in degree
        coord_in.lp.phi = msg_gps->latitude;   // latitude in degree
        
        PJ_COORD coord_out = proj_trans(projector, PJ_FWD, coord_in);

        // Conversion for origin
        Eigen::Vector4f origin( coord_out.xyz.x,
                                coord_out.xyz.y,
                                coord_out.xyz.z,
                                1.0
                               );

        Eigen::Quaternionf orientation(msg_odometry->pose.pose.orientation.x,
                                       msg_odometry->pose.pose.orientation.y,
                                       msg_odometry->pose.pose.orientation.z,
                                       msg_odometry->pose.pose.orientation.w
                                       );

        
        pcl::PCDWriter writer;
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(dummy_pointcloud, cloud2);

        std::string pcd_header = writer.generateHeaderASCII (cloud2, origin, orientation);
        outfile << pcd_header;
        point_count ++;
      }
      //append pointcloud msg on outfile
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(*msg, cloud);
      for (auto point: cloud.points)
      {
        outfile << point;
      }
      // outfile << cloud.points;

      // update point_count & width info in header
      dummy_pointcloud.width = cloud.width;
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subs_pointcloud_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subs_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subs_odom_;

    // rclcpp::Subscription<diagnostics_msgs::msg::DiagnosticArray>::SharedPtr sub_;
    // std::shared_ptr<rclcpp::Node> node_;

    pcl::PointCloud<pcl::PointXYZI> dummy_pointcloud;
    std::shared_ptr<rclcpp::Node> node_gps_;
    std::shared_ptr<rclcpp::Node> node_odom_;
    uint64_t point_count = 0;    
    PJ_CONTEXT *ctx;
    PJ *projector;
    PJ *norm;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<PointCloud2PCD> pc2pcd = std::make_shared<PointCloud2PCD>();
  rclcpp::spin(pc2pcd);
  // User hit Ctrl+C, saving mesh and shutting down...
  rclcpp::shutdown();
  pc2pcd->outfile.close();
}
