# libkinect
add_library(libkinect SHARED kinect.cpp kinect.hpp)
target_include_directories(libkinect PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(libkinect ${freenect2_LIBRARIES})
target_link_libraries(libkinect ${OpenCV_LIBS})
target_link_libraries(libkinect ${PCL_LIBRARIES})

# libpointCloud
add_library(libpointCloud SHARED pointCloud.cpp pointCloud.hpp)
target_include_directories(libpointCloud PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(libpointCloud ${PCL_LIBRARIES})
target_link_libraries(libpointCloud ${realsense2_LIBRARY})