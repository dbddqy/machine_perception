# =========================
# add libraries
# =========================

add_library(utility SHARED utility.cpp header/utility.hpp)
target_include_directories(utility PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/header")

add_library(lib_rs SHARED lib_rs.cpp header/lib_rs.hpp)
target_include_directories(lib_rs PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/header")

add_library(lib_camera SHARED header/lib_camera.hpp)
set_target_properties(lib_camera PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(lib_camera PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/header")

add_library(lib_det SHARED lib_det.cpp header/lib_det.hpp)
target_include_directories(lib_det PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/header")

add_library(lib_geometry SHARED lib_geometry.cpp header/lib_geometry.hpp)
target_include_directories(lib_geometry PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/header")

