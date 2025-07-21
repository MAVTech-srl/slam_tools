#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/types.h>
#include <pcl/point_types.h>
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
#include <string>
#include <memory>

using std::placeholders::_1;

using namespace sensor_msgs::msg;
using namespace nav_msgs::msg;
typedef message_filters::sync_policies::ApproximateTime<PointCloud2, NavSatFix, Odometry> policy;

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


      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      //Initialize subscribers
      sub_pointcloud_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/cloud_registered", qos_profile);
      sub_local_pos_ = std::make_shared<message_filters::Subscriber<Odometry>>(this, "/mavros/local_position/odom", qos_profile);
      sub_gps_ = std::make_shared<message_filters::Subscriber<NavSatFix>>(this, "/mavros/global_position/raw/fix", qos_profile);
      sync_ = std::make_shared<message_filters::Synchronizer<policy>>(policy(10), *sub_pointcloud_, *sub_gps_, *sub_local_pos_);
      sync_ -> registerCallback(&PointCloud2PCD::pointCloudCallback, this);
      
      pcl::PointCloud<pcl::PointXYZI> dummy_pointcloud;
      this->dummy_pointcloud.height = 0;
      this->dummy_pointcloud.width = 0;
      this->dummy_pointcloud.is_dense = false;
      this->dummy_pointcloud.resize(0);
      
      // BUG: Correct this file location
      this->declare_parameter("pcd_output_path", "/workspaces/fast-lio-slam-ros2/rosbag/pointcloud_debug.pcd");                                          //changed /temp/ with rosbag folder
      this->filename = this->get_parameter("pcd_output_path").as_string();
      RCLCPP_INFO(this->get_logger(), "filename : %s", this->filename.c_str());
    
      outfile.open(this->filename, std::ios::app);

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

    void update_pcd(void)
    {
      std::ifstream old_pcd_file;
      old_pcd_file.open(this->filename);

      std::string line;
      std::ostringstream updated_pcd_content;
      int line_number = 0;
      if (old_pcd_file.is_open())
      {
        while (std::getline(old_pcd_file, line))
        {
          line_number ++;
          if (line_number==7)
          {
            updated_pcd_content << "WIDTH " << point_count << std::endl;
            RCLCPP_INFO(this->get_logger(), "Updated .pcd width: %d", point_count);
          }
          else if (line_number==10)
          {
            updated_pcd_content << "POINTS " << point_count << std::endl;
            RCLCPP_INFO(this->get_logger(), "Updated .pcd number of points: %d", point_count);
          }
          else
          {
            updated_pcd_content << line << std::endl;
          }
        }
        old_pcd_file.close();
        RCLCPP_INFO(this->get_logger(), "File read and closed");
      }
      // Now open the same file in write mode to erase content and put the updated pcd
      std::ofstream updated_pcd_file(this->filename, std::ofstream::out | std::ofstream::trunc);
      if (updated_pcd_file.is_open())
      {
        updated_pcd_file << updated_pcd_content.str();
        updated_pcd_file.close();
      }
    }

    // Public variables
    std::ofstream outfile;
    std::string filename;
    uint64_t point_count = 0;

  private:
    void pointCloudCallback(const PointCloud2::SharedPtr msg_pc,
                            const NavSatFix::SharedPtr msg_gps,
                            const Odometry::SharedPtr msg_odometry)
    {
      if ( point_count == 0 )
      {
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
        pcd_header.append("DATA ascii\n");  // This is needed otherwise pcl_viewer give "malformed pcd file" warning
        
        if (outfile.is_open())
        {
          outfile << pcd_header;
          outfile.flush();
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Pointcloud file is not open!");
        }
      }
      //append pointcloud msg on outfile
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(*msg_pc, cloud);
      for (auto point: cloud.points)
      {
        outfile << point.x << " " << point.y << " " << point.z << " " << point.intensity << std::endl;
      }
      // update point_count & width info
      point_count += cloud.height * cloud.width;
      dummy_pointcloud.width = cloud.width;
    }
    std::shared_ptr<message_filters::Subscriber<PointCloud2>> sub_pointcloud_;
    std::shared_ptr<message_filters::Subscriber<NavSatFix>> sub_gps_;
    std::shared_ptr<message_filters::Subscriber<Odometry>> sub_local_pos_;
    std::shared_ptr<message_filters::Synchronizer<policy>> sync_;

    NavSatFix::SharedPtr msg_gps;
    Odometry::SharedPtr msg_odometry;

    pcl::PointCloud<pcl::PointXYZI> dummy_pointcloud;
    std::shared_ptr<rclcpp::Node> node_gps_;
    std::shared_ptr<rclcpp::Node> node_odom_;
       
    PJ_CONTEXT *ctx;
    PJ *projector;
    PJ *norm;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto default_context = rclcpp::contexts::get_global_default_context();
  std::shared_ptr<PointCloud2PCD> pc2pcd = std::make_shared<PointCloud2PCD>();
  rclcpp::Context::PreShutdownCallback shutdown_callback = [&](){
    RCLCPP_INFO(pc2pcd->get_logger(), "Performing pre-shutting down actions...");
    pc2pcd->outfile.close();
    pc2pcd->update_pcd();
  };
  default_context->add_pre_shutdown_callback(shutdown_callback);

  rclcpp::spin(pc2pcd);
  rclcpp::shutdown();

}
