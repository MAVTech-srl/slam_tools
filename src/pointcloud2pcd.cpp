#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/types.h>
#include <pcl/point_types.h>

using std::placeholders::_1;

using namespace sensor_msgs::msg;

class PointCloud2PCD : public rclcpp::Node
{
  public:
    PointCloud2PCD() 
    : Node("pointcloud2pcd") 
    {
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/cloud_registered", qos, std::bind(&PointCloud2PCD::pointCloudCallback, this, _1));

      pcl::PointCloud<pcl::PointXYZI> dummy_pointcloud;
      this->dummy_pointcloud.height = 0;
      this->dummy_pointcloud.width = 0;
      this->dummy_pointcloud.is_dense = false;
      this->dummy_pointcloud.resize(0);
      
      // BUG: Correct this file location
      this->declare_parameter("pcd_output_path", "/workspaces/fast-lio-slam-ros2/rosbag/pointcloud_local_debug.pcd");                                          //changed /temp/ with rosbag folder
      this->filename = this->get_parameter("pcd_output_path").as_string();
      RCLCPP_INFO(this->get_logger(), "filename : %s", this->filename.c_str());
    
      outfile.open(this->filename, std::ios::app);
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

    std::ofstream outfile;
    std::string filename;
    uint64_t point_count = 0;
  
  private:
    void pointCloudCallback(const PointCloud2::SharedPtr msg_pc)
    {
      if ( point_count == 0 )
      {
        // Conversion for origin
        Eigen::Vector4f origin( 0.0,
                                0.0,
                                0.0,
                                1.0
                               );

        Eigen::Quaternionf orientation(0.0,
                                       0.0,
                                       0.0,
                                       1.0
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
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    pcl::PointCloud<pcl::PointXYZI> dummy_pointcloud;
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<PointCloud2PCD> pc2pcd = std::make_shared<PointCloud2PCD>();

  rclcpp::spin(pc2pcd);
  rclcpp::shutdown();
  
  RCLCPP_INFO(pc2pcd->get_logger(), "Performing pre-shutting down actions...");
  pc2pcd->outfile.close();
  pc2pcd->update_pcd();
}
