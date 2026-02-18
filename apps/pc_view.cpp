#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: pc_view <file.pcd> [leaf_size]\n"
                  << "  leaf_size: voxel size for downsampling (default 0.5)\n";
        return 1;
    }

    const std::string path = argv[1];
    const float leaf = (argc >= 3) ? std::stof(argv[2]) : 0.5f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0) {
        std::cerr << "Error: failed to load PCD file: " << path << "\n";
        return 1;
    }

    // Remove NaNs
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);

    // Downsample:
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*ds);

    std::cout << "Loaded points: " << cloud->size() << "\n";
    std::cout << "Downsampled (leaf " << leaf << "): " << ds->size() << "\n";

    // Compute centroid in double

    Eigen::Vector4d c;
    pcl::compute3DCentroid(*ds, c);

    //Shift to local coordinates around centroid:
    pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>());
    local->reserve(ds->size());
    for (const auto& p : ds->points) {
        local->push_back(pcl::PointXYZ(
            static_cast<float>(p.x - c[0]),
            static_cast<float>(p.y - c[1]),
            static_cast<float>(p.z - c[2])
        ));
    }
    local->width = static_cast<uint32_t>(local->size());
    local->height = 1;
    local->is_dense = ds->is_dense;

    auto vis = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("pc_view (local frame)")
    );
    vis->setBackgroundColor(0,0,0);

    vis->addPointCloud<pcl::PointXYZ>(local, "cloud");
    vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud"
    );

    // Coordinate axes in local units, aim camera at the cloud

    vis->addCoordinateSystem(10.0);
    vis->resetCameraViewpoint("cloud");
    vis->resetCamera();

    while (!vis->wasStopped()) {
        vis->spinOnce(16);
    }

    return 0;
}
