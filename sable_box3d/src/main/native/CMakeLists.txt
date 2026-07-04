cmake_minimum_required(VERSION 3.20)
project(box3d_jni)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

find_package(JNI REQUIRED)

add_library(box3d_jni SHARED
    box3d_jni.cpp
)

target_include_directories(box3d_jni PRIVATE
    ${JNI_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

add_library(box3d STATIC IMPORTED)

set_target_properties(box3d PROPERTIES
    IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/box3d.lib
)

target_link_libraries(box3d_jni PRIVATE box3d)