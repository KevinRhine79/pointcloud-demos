# pointcloud-demos

C++17 demos using PCL (Point Cloud Library).

## What’s included
- `pc_stats`: loads a `.pcd` point cloud and prints point count, AABB bounds, and centroid
- `pc_view`: loads a `.pcd`, removes NaNs, voxel-downsamples, shifts to a local frame, and visualizes the cloud
- `pc_segmentation`: loads a `.pcd`, removes NaNs, fits a dominant plane using RANSAC, and saves a colored output cloud with plane inliers highlighted

## Build
```bash
cmake -S . -B build
cmake --build build -j
```

## Run
```bash
./build/pc_stats your_cloud.pcd
./build/pc_view your_cloud.pcd 0.5
./build/pc_segmentation your_cloud.pcd 1.0
```

- `pc_view` optional argument: `leaf_size` = voxel size for downsampling (default: `0.5`)
- `pc_segmentation` optional argument: `tolerance` = maximum planar distance for determining inliers

## Output
`pc_segmentation` saves a colored `.pcd` file to:
```text
sample_pointclouds/segmented_pointclouds/<name>_segmented.pcd
```

## Notes
- `pc_view` currently works as a lightweight visualization tool for raw or processed point clouds.
- `pc_segmentation` currently performs single-plane segmentation using `pcl::SACSegmentation` with a plane model and RANSAC

## Environment
- Tested on macOS (Apple Silicon) with PCL installed via Homebrew