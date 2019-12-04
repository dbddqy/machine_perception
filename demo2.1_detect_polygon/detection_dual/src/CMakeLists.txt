add_executable(main main.cpp)

target_link_libraries(main PRIVATE libdetection)
target_link_libraries(main PRIVATE libmatching)
target_link_libraries(main PRIVATE libcameraConfig)
