project (detection)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
cmake_minimum_required(VERSION 3.15)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(realsense2 REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

#find_package( PCL REQUIRED COMPONENT common io )
#include_directories( ${PCL_INCLUDE_DIRS} )
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
#add_definitions( ${PCL_DEFINITIONS} )

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#new
add_executable(takeFrame2D takeFrame2D.cpp)
target_link_libraries(takeFrame2D ${OpenCV_LIBS} ${realsense2_LIBRARY})

add_executable(construct3D construct3D.cpp)
target_link_libraries(construct3D ${OpenCV_LIBS} ${realsense2_LIBRARY} ${PCL_LIBRARIES})

add_executable(takeFrame3D takeFrame3D.cpp)
target_link_libraries(takeFrame3D ${OpenCV_LIBS} ${realsense2_LIBRARY} ${PCL_LIBRARIES})

add_executable(registerClouds registerClouds.cpp)
target_link_libraries(registerClouds ${PCL_LIBRARIES})

add_executable(ransac ransac.cpp)
target_link_libraries(ransac ${PCL_LIBRARIES})

add_executable(clean_cloud clean_cloud.cpp)
target_link_libraries(clean_cloud ${PCL_LIBRARIES})

add_executable(downsample downsample.cpp)
target_link_libraries(downsample ${PCL_LIBRARIES})

add_executable(passthrough passthrough.cpp)
target_link_libraries(passthrough ${PCL_LIBRARIES})

add_executable(combine_cloud combine_cloud.cpp)
target_link_libraries(combine_cloud ${PCL_LIBRARIES})

add_executable(fitting fitting.cpp)
target_link_libraries(fitting ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#target_link_libraries(main ${OpenCV_LIBS})
#target_link_libraries(main ${freenect2_LIBRARIES})
#target_link_libraries(main Eigen3::Eigen)
#target_link_libraries(main ${Pangolin_LIBRARIES})
#target_link_libraries(main ${PCL_LIBRARIES})
