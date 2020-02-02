# libkinect
add_library(libkinect SHARED kinect.cpp kinect.hpp)
target_include_directories(libkinect PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(libkinect ${freenect2_LIBRARIES})
target_link_libraries(libkinect ${OpenCV_LIBS})
target_link_libraries(libkinect ${PCL_LIBRARIES})

# libpose
add_library(libpose SHARED pose.cpp pose.hpp)
target_include_directories(libpose PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(libpose ${OpenCV_LIBS})

# libvision
add_library(libvision SHARED vision.cpp vision.hpp)
target_include_directories(libvision PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(libvision ${OpenCV_LIBS})
#target_link_libraries(vision ${PCL_LIBRARIES})
#target_link_libraries(vision ${realsense2_LIBRARY})

# libpointCloud
add_library(libpointCloud SHARED pointCloud.cpp pointCloud.hpp)
target_include_directories(libpointCloud PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(libpointCloud ${PCL_LIBRARIES})
target_link_libraries(libpointCloud ${realsense2_LIBRARY})
target_link_libraries(libpointCloud ${OpenCV_LIBS})

link_libraries(libvision PUBLIC libpose)
link_libraries(libpointCloud PUBLIC libvision)
