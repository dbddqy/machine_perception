project(detection)

set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
cmake_minimum_required(VERSION 3.15)

# =========================
# include packages
# =========================

find_package(realsense2 REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# =========================
# add sub-directories
# =========================

add_subdirectory(lib)
add_subdirectory(src)
