#pragma once

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/types.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
// using PointCloud = PointCloud<PointXYZ>;

namespace tools
{
    void greedy_projection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud);
}