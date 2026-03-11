#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: pc_view <file.pcd> [leaf_size]\n"
                  << "  leaf_size: voxel size for downsampling (default: 0.5)\n";
        return 1;
    }

    const std::string path = argv[1];
    const float leaf = (argc >= 3) ? std::stof(argv[2]) : 0.5f;

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) != 0) {
        std::cerr << "Error: failed to load PCD file: " << path << "\n";
        return 1;
    }

    // Remove NaNs
    std::cout << "Checking for NaNs...\n";
    int before = cloud->size();
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);
    if (before - cloud->size() != 0) {
        std::cout << "Removed " << before - cloud->size() << " points.\n";
    }
    else std::cout << "No NaNs found.\n";

    // Downsample:
    std::cout << "Downsampling with leaf size: " << leaf << "\n";
    auto ds = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*ds);

    std::cout << "Loaded points: " << cloud->size() << "\n";
    std::cout << "Downsampled points: " << ds->size() << "\n";

    // Compute centroid in double

    Eigen::Vector4d c;
    pcl::compute3DCentroid(*ds, c);

    //Shift to local coordinates around centroid:
    auto local = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    local->reserve(ds->size());
    for (const auto& p : ds->points) {
        pcl::PointXYZRGB q;

        q.x = static_cast<float>(p.x - c[0]);
        q.y = static_cast<float>(p.y - c[1]);
        q.z = static_cast<float>(p.z - c[2]);

        q.r = p.r;
        q.g = p.g;
        q.b = p.b;

        local->push_back(q);
    }

    local->width = static_cast<uint32_t>(local->size());
    local->height = 1;
    // local->is_dense = ds->is_dense;

    auto vis = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("pc_view (local frame)")
    );
    vis->setBackgroundColor(0,0,0);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(local);
    vis->addPointCloud<pcl::PointXYZRGB>(local, rgb, "cloud");
    vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud"
    );

    // Coordinate axes in local units, aim camera at the cloud

    vis->resetCameraViewpoint("cloud");
    vis->resetCamera();

    while (!vis->wasStopped()) {
        vis->spinOnce(16);
    }

    return 0;
}
