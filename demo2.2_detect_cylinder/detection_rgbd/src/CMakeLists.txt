add_executable(main main.cpp)
target_link_libraries(main PRIVATE libkinect)
target_link_libraries(main PRIVATE libpointCloud)
# link librealsense2 package
target_link_libraries(main PRIVATE ${realsense2_LIBRARY})

add_executable(takeFrameKinect takeFrameKinect.cpp)
target_link_libraries(takeFrameKinect PRIVATE libkinect)

add_executable(takeFrameRealsense takeFrameRealsense.cpp)
target_link_libraries(takeFrameRealsense PRIVATE libpointCloud)
target_link_libraries(takeFrameRealsense PRIVATE ${realsense2_LIBRARY})
target_link_libraries(takeFrameRealsense PRIVATE ${OpenCV_LIBS})
target_link_libraries(takeFrameRealsense PRIVATE ${PCL_LIBRARIES})

add_executable(test test.cpp)
target_link_libraries(test PRIVATE libpointCloud)
target_link_libraries(test PRIVATE ${PCL_LIBRARIES})

add_executable(fitting fitting.cpp)
target_link_libraries(fitting PRIVATE libpointCloud)
target_link_libraries(fitting PRIVATE ${PCL_LIBRARIES} ${CERES_LIBRARIES})
