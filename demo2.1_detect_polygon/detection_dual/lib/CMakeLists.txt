add_library(libdetection SHARED detection.cpp)
add_library(libmatching SHARED matching.cpp)
add_library(libcameraConfig SHARED CameraConfig.cpp)
add_library(libPose SHARED Pose.cpp)

target_include_directories(libdetection PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(libmatching PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(libcameraConfig PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(libPose PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

link_libraries(libdetection PRIVATE libcameraConfig)
link_libraries(libmatching PRIVATE libPose)