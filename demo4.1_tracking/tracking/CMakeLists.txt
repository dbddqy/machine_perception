project (tracking)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
cmake_minimum_required(VERSION 3.15)

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

#new
add_executable(main main.cpp)
#target_link_libraries(main ${OpenCV_LIBS} ${realsense2_LIBRARY} ${CERES_LIBRARIES})
target_link_libraries(main ${OpenCV_LIBS} ${realsense2_LIBRARY} ${PCL_LIBRARIES} ${CERES_LIBRARIES})
