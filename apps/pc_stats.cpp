#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

int main(int argc, char** argv) {
    // Ensure command line argument is non-empty
    if (argc < 2) {
        std::cerr << "Usage: pc_stats <file.pcd>\n";
        return 1;
    }

    const std::string path = argv[1];

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Ensure that a .pcd file is successfully loaded
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0) {
        std::cerr << "Error: failed to load PCD file: " << path << "\n";
        return 1;
    }

    std::cout << "Loaded: " << path << "\n";
    std::cout << "Points: " << cloud->size() << "\n";

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    std::cout << "AABB min: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")\n";
    std::cout << "AABB max: (" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")\n";

    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    std::cout << "Centroid: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")\n";

    return 0;
}
