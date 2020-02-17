project (cylinder_2.0)
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

add_executable(takeFrameRealsense takeFrameRealsense.cpp)
target_link_libraries(takeFrameRealsense ${OpenCV_LIBS} ${realsense2_LIBRARY} ${PCL_LIBRARIES})

add_executable(registerClouds registerClouds.cpp)
target_link_libraries(registerClouds ${PCL_LIBRARIES})

#target_link_libraries(main ${OpenCV_LIBS})
#target_link_libraries(main ${freenect2_LIBRARIES})
#target_link_libraries(main Eigen3::Eigen)
#target_link_libraries(main ${Pangolin_LIBRARIES})
#target_link_libraries(main ${PCL_LIBRARIES})
