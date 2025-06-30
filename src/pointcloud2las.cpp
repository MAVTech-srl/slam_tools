#include <cstdio>
#include "rclcpp/rclcpp.hpp"  
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// MAVROS message definitions
#include "sensor_msgs/msg/nav_sat_fix.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "px4_msgs/msg/sensor_gps.hpp"
// #include "px4_msgs/msg/vehicle_local_position.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <rclcpp/qos.hpp>

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasWriter.hpp>

#include <cassert>
#include <cmath>   // for HUGE_VAL
#include <iomanip> // for std::setprecision()

#include <iostream>

#include "proj.h"

using std::placeholders::_1;

using namespace sensor_msgs::msg;
// using namespace geometry_msgs::msg;
// using namespace nav_msgs::msg;
using namespace message_filters;

// typedef message_filters::sync_policies::ApproximateTime<PointCloud2, Odometry> policy;
typedef message_filters::sync_policies::ApproximateTime<PointCloud2, NavSatFix> policy;
class PointCloud2LAS : public rclcpp::Node
{
  public:
    PointCloud2LAS() 
    : Node("PointCloud2LAS"),
      view(new pdal::PointView(table)),
      // writer(factory->createStage("writer.las")),
      ctx(PJ_DEFAULT_CTX)
    {
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
      
      //Initialize subscribers
      sub_pointcloud_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/cloud_registered", qos_profile);
      // sub_gps_ = std::make_shared<message_filters::Subscriber<NavSatFix>>(this, "/mavros/global_position/global", qos_profile);
      // sub_local_pos_ = std::make_shared<message_filters::Subscriber<Odometry>>(this, "/mavros/global_position/local", qos_profile);
      sub_gps_ = std::make_shared<message_filters::Subscriber<NavSatFix>>(this, "/mavros/global_position/raw/fix", qos_profile);
      sync_ = std::make_shared<message_filters::Synchronizer<policy>>(policy(10), *sub_pointcloud_, *sub_gps_);
      sync_ -> registerCallback(&PointCloud2LAS::Callback, this);

      this->declare_parameter("las_output_path", "/workspaces/fast-lio-slam-ros2/rosbag/pointcloud.las");                                          //changed /temp/ with rosbag folder
      std::string filename = this->get_parameter("las_output_path").as_string();


      options.add("filename", filename);

      table.layout()->registerDim(pdal::Dimension::Id::X);
      table.layout()->registerDim(pdal::Dimension::Id::Y);
      table.layout()->registerDim(pdal::Dimension::Id::Z);

      // reader.addView(view);
      // writer->setInput(reader);
      // writer->setOptions(options);
      // writer->prepare(table);

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
    void save_map(void)
    {
      //
    }
    pcl::PointCloud<pcl::PointXYZ> map;


  
  private:
    void Callback(const PointCloud2::SharedPtr msg_pointcloud, 
                  const NavSatFix::SharedPtr msg_gps)
                  // const Odometry::SharedPtr msg_pos)
    {
      RCLCPP_INFO(this->get_logger(), "Entered Callback");                                                      //print statement to verify callback
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*msg_pointcloud, cloud);
      // map += cloud;

      // Perform the coordinate transformation.

      PJ_COORD coord_in;
      coord_in.lpzt.z = msg_gps->altitude;            // z ordinate. unused (altitude already in meters)
      coord_in.lpzt.t = HUGE_VAL;       // time ordinate. unused
      coord_in.lp.lam = msg_gps->longitude;   // longitude in degree
      coord_in.lp.phi = msg_gps->latitude;   // latitude in degree
      

      PJ_COORD coord_out = proj_trans(projector, PJ_FWD, coord_in);

      // Display result
      std::cout << std::fixed << std::setprecision(3);
      std::cout << "Easting: " << coord_out.enu.e << std::endl;  
      std::cout << "Northing: " << coord_out.enu.n << std::endl; 

      // Add points to las file in UTM format
      // while (true)
      for (auto point : cloud.points)
      {
        view->setField(pdal::Dimension::Id::X, point_count, coord_out.xyz.x + point.x * sin(coord_in.opk.k - M_PI));  //old version: msg_pos->heading
        view->setField(pdal::Dimension::Id::Y, point_count, coord_out.xyz.y + point.y * cos(coord_in.opk.k - M_PI));  //opk_k refers to third angle-kappa
        view->setField(pdal::Dimension::Id::Z, point_count, coord_out.xyz.z + point.z);
        point_count++;
      }

    // NOTE: NOT NEEDED if /mavros/global_position/local will be used as it is already in UTM!

    // // Convert quaternion to RPY angles
    // tf2::Quaternion quat(   msg_pos->pose.pose.orientation.x,
    //                         msg_pos->pose.pose.orientation.y,
    //                         msg_pos->pose.pose.orientation.z,
    //                         msg_pos->pose.pose.orientation.w
    //                     );
    // tf2::Matrix3x3 rotation_mat(quat);
    // double roll, pitch, yaw;
    // rotation_mat.getRPY(roll, pitch, yaw);
    
    // // Add points to las file in UTM format
    //   for (auto point : cloud.points)
    //   {
    //     view->setField(pdal::Dimension::Id::X, point_count, msg_pos->pose.pose.position.x + point.x * sin(yaw - M_PI));
    //     view->setField(pdal::Dimension::Id::Y, point_count, msg_pos->pose.pose.position.y + point.y * cos(yaw - M_PI));
    //     view->setField(pdal::Dimension::Id::Z, point_count, msg_pos->pose.pose.position.z + point.z);
    //     point_count++;
    //   }

      reader.addView(view);
      writer->setInput(reader);
      writer->setOptions(options);
      writer->prepare(table);
      writer->execute(table);
      RCLCPP_INFO(this->get_logger(), "Saved and exit Callback");
      // cloud.points.
      // save_map(); 

    //   // std::string filename = "/tmp/pointcloud.pcd";
    //   // if (pcl::io::savePCDFileASCII(filename, cloud)==0) {
    //   //   RCLCPP_INFO(this->get_logger(), "Saved PCD file to %s", filename.c_str());
    //   // } else {
    //   //   RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file.");
    //   // }
    }

    std::shared_ptr<message_filters::Subscriber<PointCloud2>> sub_pointcloud_;
    std::shared_ptr<message_filters::Subscriber<NavSatFix>> sub_gps_;
    // std::shared_ptr<message_filters::Subscriber<Odometry>> sub_local_pos_;
    std::shared_ptr<message_filters::Synchronizer<policy>> sync_;
    pdal::Options options;
    pdal::PointTable table;
    pdal::PointViewPtr view;
    size_t point_count = 0;
    pdal::BufferReader reader;
    pdal::StageFactory *factory = new pdal::StageFactory;
    // pdal::Stage *writer = new pdal::Stage;
    pdal::LasWriter *writer = new pdal::LasWriter;

    PJ_CONTEXT *ctx;
    PJ *projector;
    PJ *norm;
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<PointCloud2LAS> pc2las = std::make_shared<PointCloud2LAS>();
  rclcpp::spin(pc2las);
  // User hit Ctrl+C, saving mesh and shutting down...
  rclcpp::shutdown();
}
