#include <cstdio>
#include "rclcpp/rclcpp.hpp"  
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>

#include <cassert>
#include <cmath>   // for HUGE_VAL
#include <iomanip> // for std::setprecision()

#include <iostream>

#include "proj/coordinateoperation.hpp"
#include "proj/crs.hpp"
#include "proj/io.hpp"
#include "proj/util.hpp" // for nn_dynamic_pointer_cast



using std::placeholders::_1;

using namespace NS_PROJ::crs;
using namespace NS_PROJ::io;
using namespace NS_PROJ::operation;
using namespace NS_PROJ::util;

using namespace sensor_msgs;
using namespace message_filters;

class PointCloud2LAS : public rclcpp::Node
{
  public:
    PointCloud2LAS() 
    : Node("PointCloud2LAS"),
      view(new pdal::PointView(table))
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/cloud_registered", 1, std::bind(&PointCloud2LAS::pointCloudCallback, this, _1));
      subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/raw_gps_data", 1, std::bind(&PointCloud2LAS::pointCloudCallback, this, _1));

      this->declare_parameter("las_output_path", "/tmp/pointcloud.las");
      std::string filename = this->get_parameter("las_output_path").as_string();
      
      options.add("filename", filename);

      table.layout()->registerDim(pdal::Dimension::Id::X);
      table.layout()->registerDim(pdal::Dimension::Id::Y);
      table.layout()->registerDim(pdal::Dimension::Id::Z);

      reader.addView(view);
      writer = factory.createStage("writer.las");
      writer->setInput(reader);
      writer->setOptions(options);
      writer->prepare(table);

    }
    void save_map(void)
    {
      
    //   if (pcl::io::savePCDFileASCII(filename, map) == 0) {
    //     RCLCPP_INFO(this->get_logger(), "Saved PCD file to %s", filename.c_str());
    //   } else {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file.");
    //   }
    //   // this->create_mesh();
    }
    // void create_mesh(void)
    // {
    //   tools::greedy_projection(this->map.makeShared());
    // }
    pcl::PointCloud<pcl::PointXYZ> map;
  
  private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*msg, cloud);
      map += cloud;

      // cloud.points.
      // save_map(); 

    //   // std::string filename = "/tmp/pointcloud.pcd";
    //   // if (pcl::io::savePCDFileASCII(filename, cloud)==0) {
    //   //   RCLCPP_INFO(this->get_logger(), "Saved PCD file to %s", filename.c_str());
    //   // } else {
    //   //   RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file.");
    //   // }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    pdal::Options options;
    pdal::PointTable table;
    pdal::PointViewPtr view;
    size_t point_count = 0;
    pdal::BufferReader reader;
    pdal::StageFactory factory;
    pdal::Stage *writer;
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<PointCloud2LAS> pc2las = std::make_shared<PointCloud2LAS>();
  rclcpp::spin(pc2las);
  // User hit Ctrl+C, saving mesh and shutting down...
  rclcpp::shutdown();
}
