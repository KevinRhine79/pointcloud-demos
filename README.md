# pointcloud-demos

C++11 demos using PCL (Point Cloud Library).

## What’s included
- `pc_stats`: loads a `.pcd` point cloud and prints point count, AABB bounds, and centroid
- `pc_view`: loads a `.pcd`, removes NaNs, voxel-downsamples, shifts to a local frame, and visualizes the cloud

## Build
```bash
cmake -S . -B build
cmake --build build -j
```

## Run
```bash
./build/pc_stats your_cloud.pcd
./build/pc_view your_cloud.pcd 0.5
```

- `pc_view` optional argument: `leaf_size` = voxel size for downsampling (default: `0.5`)

## Environment
- Tested on macOS (Apple Silicon) with PCL installed via Homebrew