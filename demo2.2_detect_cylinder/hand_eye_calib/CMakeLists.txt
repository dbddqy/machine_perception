project (hand_eye_calib)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
cmake_minimum_required(VERSION 3.15)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(realsense2 REQUIRED)

add_executable(hand_eye_calib hand_eye_calib.cpp)
target_link_libraries(hand_eye_calib PRIVATE ${CERES_LIBRARIES} Eigen3::Eigen)

add_executable(detect_marker detect_marker.cpp)
target_link_libraries(detect_marker PRIVATE ${OpenCV_LIBS} ${realsense2_LIBRARY})
