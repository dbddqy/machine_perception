cmake_minimum_required(VERSION 3.14)
project(detection_aruco)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(lib)
add_subdirectory(src)
#include_directories(lib)

#link opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(main PRIVATE ${OpenCV_LIBS})
target_link_libraries(libdetection ${OpenCV_LIBS})
target_link_libraries(libmatching ${OpenCV_LIBS})
target_link_libraries(libcameraConfig ${OpenCV_LIBS})
target_link_libraries(libPose ${OpenCV_LIBS})

#link rapidjson
find_package(RapidJSON REQUIRED)
include_directories(${RapidJSON_INCLUDE_DIRS})