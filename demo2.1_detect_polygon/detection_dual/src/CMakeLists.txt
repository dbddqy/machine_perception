add_executable(main main.cpp main.hpp)

target_link_libraries(main PRIVATE libdetection)
target_link_libraries(main PRIVATE libmatching)
target_link_libraries(main PRIVATE libcameraConfig)
target_link_libraries(main PRIVATE libPose)
