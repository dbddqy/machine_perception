add_executable(take_frame take_frame.cpp)

target_link_libraries(take_frame lib_rs ${OpenCV_LIBS} ${realsense2_LIBRARY})
