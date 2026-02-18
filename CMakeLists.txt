cmake_minimum_required(VERSION 3.16)
project(pointcloud_demos LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL REQUIRED COMPONENTS common io)

add_executable(pc_stats apps/pc_stats.cpp)
target_include_directories(pc_stats PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pc_stats PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(pc_stats PRIVATE ${PCL_DEFINITIONS})

find_package(PCL REQUIRED COMPONENTS common io filters visualization)

add_executable(pc_view apps/pc_view.cpp)
target_include_directories(pc_view PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pc_view PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(pc_view PRIVATE ${PCL_DEFINITIONS})
