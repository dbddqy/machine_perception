# =========================
# set library dependencies
# =========================

set(LIB_RS_DEP lib_rs ${OpenCV_LIBS} ${realsense2_LIBRARY})
set(LIB_CAMERA_DEP lib_camera yaml-cpp ${OpenCV_LIBS})
set(LIB_DET_DEP lib_det lib_geometry utility ${LIB_CAMERA_DEP} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# ============================
# add executable and link libs
# ============================

add_executable(take_frame take_frame.cpp)
target_link_libraries(take_frame ${LIB_RS_DEP} ${LIB_DET_DEP})

add_executable(detect detect.cpp)
target_link_libraries(detect ${LIB_DET_DEP})

#add_executable(test test.cpp)
#target_link_libraries(test ${LIB_CAMERA_DEP})

add_executable(assemble assemble.cpp)
target_link_libraries(assemble ${LIB_DET_DEP})
