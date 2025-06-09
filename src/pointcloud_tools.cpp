#include <pointcloud_tools.hpp>

using namespace pcl;

namespace tools 
{
    void greedy_projection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud)
    {
        std::cerr << "Creating mesh..." << std::endl;
        // Downsample cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

        // Denoise cloud
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (input_cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter(*cloud_filtered);

        std::cerr << "Downsampling..." << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vxg;
        vxg.setInputCloud (cloud_filtered);
        vxg.setLeafSize (0.1f, 0.1f, 0.1f);
        vxg.filter (*cloud_downsampled);


        // Smooth cloud
        //BUG --> IT DIES HERE!!
        std::cerr << "Smoothing..." << std::endl;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

        mls.setComputeNormals (true);
        // Set parameters
        mls.setInputCloud (cloud_downsampled);
        mls.setPolynomialOrder (2);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.03);

        // Reconstruct
        mls.process (*mls_points);
        
        // Normal estimation*
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

        // tree->setInputCloud (cloud_filtered);
        // n.setInputCloud (cloud_filtered);
        // n.setSearchMethod (tree);
        // // n.setKSearch (20);
        // n.setRadiusSearch(0.05);
        // n.compute (*normals);
        // //* normals should not contain the point normals + surface curvatures


        // // Concatenate the XYZ and normal fields*
        // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        // pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
        //* cloud_with_normals = cloud + normals


        // Create search tree*
        std::cerr << "Meshing..." << std::endl;
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        // tree2->setInputCloud (cloud_with_normals);
        tree2->setInputCloud (mls_points);


        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;


        // Set the maximum distance between connected points (maximum edge length)
        // gp3.setSearchRadius (0.025);
        gp3.setSearchRadius (0.1);


        // Set typical values for the parameters
        gp3.setMu (2.5);
        gp3.setMaximumNearestNeighbors (200);
        gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        // gp3.setMinimumAngle(M_PI/18); // 10 degrees
        gp3.setMinimumAngle(M_PI/18); // 10 degrees
        gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
        gp3.setNormalConsistency(false);


        // Get result
        // gp3.setInputCloud (cloud_with_normals);
        gp3.setInputCloud (mls_points);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangles);


        // Additional vertex information
        std::vector<int> parts = gp3.getPartIDs();
        std::vector<int> states = gp3.getPointStates();

        std::cerr << "Saving mesh..." << std::endl;
        pcl::io::saveVTKFile("/tmp/mesh.vtk", triangles);
        std::cerr << "Mesh saved!" << std::endl;
    }
} // namespace tools    
