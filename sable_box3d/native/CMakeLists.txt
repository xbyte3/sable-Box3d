cmake_minimum_required(VERSION 3.20)
project(box3d_jni)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(JNI REQUIRED)
find_package(Java 21)
include(UseJava)

add_library(box3d_jni SHARED
    box3d_jni.cpp
)

target_include_directories(box3d_jni PRIVATE
    ${JNI_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(box3d_jni PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/box3dd.lib
)

target_link_libraries(box3d_jni PRIVATE
    ${BOX3D_LIBS}
)

if (NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Only x64 build supported")
endif()

target_compile_definitions(box3d_jni PRIVATE BOX3D_DOUBLE_PRECISION)

#set_property(TARGET box3d_jni PROPERTY
#    MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
#)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

target_compile_options(box3d_jni PRIVATE
    /W3
    /Zi
    /Od
)

target_link_options(box3d_jni PRIVATE
    /DEBUG
    /MAP
)

# --- Debug info ---
message(STATUS "JNI include dirs: ${JNI_INCLUDE_DIRS}")
message(STATUS "BOX3D_LIBS=${BOX3D_LIBS}")