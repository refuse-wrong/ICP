cmake_minimum_required(VERSION 3.0)
project(point_cloud_registration)
find_package(PCL 1.14.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(point_cloud_registration main.cpp)
target_link_libraries(point_cloud_registration ${PCL_LIBRARIES})

add_executable(visualize_point_clouds visualize_point_clouds.cpp)
target_link_libraries(visualize_point_clouds ${PCL_LIBRARIES})

add_executable(l2c_icp l2c_icp.cpp)
target_link_libraries(l2c_icp ${PCL_LIBRARIES})