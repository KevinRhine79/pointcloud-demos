#include <iostream>
#include <string>
#include <vector>
#include <filesystem>

#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: pc_segmentation <file.pcd>\n";
        return 1;
    }

    const std::string path = argv[1] ;
    const float tol = (argc > 2) ? std::stof(argv[2]) : 1.0f;

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if( pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0) {
        std::cerr << "Failed to load PCD file: " << path << "\n";
        return 1;
    }

    // Remove NaNs
    std::cout << "Checking for NaNs...\n";
    size_t before = cloud->size();
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);
    if (before - cloud->size() != 0) {
        std::cout << "Removed " << before - cloud->size() << " points.\n";
    }
    else std::cout << "No NaNs found.\n";

    // Perform RANSAC plane fitting
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();

    pcl::SACSegmentation<pcl::PointXYZ> seg ;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(tol);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);

    if(inliers -> indices.empty()) {
        std::cerr << "No plane found.\n" ;
        return 1;
    }

    const auto& vals = coefficients -> values;
    std::cout << "Plane: "
              << vals[0] << "x + "
              << vals[1] << "y + "
              << vals[2] << "z + "
              << vals[3] << " = 0\n";

    std::cout << "Inliers: " << inliers -> indices.size()
              << " / " << cloud -> size() << "\n" ;

    // Create a colored pointcloud

    auto colored = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    colored -> reserve(cloud->size());

    for (const auto& p : cloud -> points){
        pcl::PointXYZRGB q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.r = 255;
        q.g = 255;
        q.b = 255;
        colored -> push_back(q);
    }

    for (int idx : inliers -> indices) {
        auto& q = colored -> points[idx];
        q.r = 0;
        q.g = 255;
        q.b = 0;
    }

    colored -> width = static_cast<uint32_t>(colored->size());
    colored -> height = 1;
    colored -> is_dense = cloud -> is_dense;

    // Save colored pointcloud to output file

    fs::path input_path(path);
    fs::path out_dir = "sample_pointclouds/segmented_pointclouds";

    if(!fs::exists(out_dir)){
        fs::create_directories(out_dir);
    }

    fs::path out_path = out_dir / (input_path.stem().string() + "_segmented.pcd");

    //const std::string out_path = "output_file.pcd";
    if (pcl::io::savePCDFileBinary(out_path, *colored) != 0) {
        std::cerr << "Failed to save output file: " << out_path << "\n";
        return 1;
    }

    std::cout << "Saved colored output file to " << out_path << "\n";

    return 0;
}
