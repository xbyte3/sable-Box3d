cmake_minimum_required(VERSION 3.20)
project(box3d_jni)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# --- JNI ---
find_package(JNI REQUIRED)
find_package(Java 21)
include(UseJava)

# --- JNI native library ---
add_library(box3d_jni SHARED
    box3d_jni.cpp
 )

target_include_directories(box3d_jni PRIVATE
    ${JNI_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

# --- /lib/* ---
target_link_libraries(box3d_jni PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/box3dd.lib
)

# Link
target_link_libraries(box3d_jni PRIVATE
    ${BOX3D_LIBS}
)

target_compile_definitions(box3d_jni PRIVATE BOX3D_DOUBLE_PRECISION)

set_property(TARGET box3d_jni PROPERTY
    MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
)

target_compile_options(box3d_jni PRIVATE /W3)

# --- Debug info ---
message(STATUS "JNI include dirs: ${JNI_INCLUDE_DIRS}")
message(STATUS "Found Box3D libs:")
foreach(lib ${BOX3D_LIBS})
    message(STATUS "  ${lib}")
endforeach()