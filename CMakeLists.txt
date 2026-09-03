cmake_minimum_required(VERSION 3.24)

project(ray_tracing_engine LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BUILD_LIST core,imgcodecs CACHE STRING "OpenCV modules required by this project")
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_PERF_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_opencv_apps OFF CACHE BOOL "" FORCE)
set(BUILD_opencv_python_bindings_generator OFF CACHE BOOL "" FORCE)
set(BUILD_JAVA OFF CACHE BOOL "" FORCE)

set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)

add_subdirectory(external/opencv-5.0.0 external/opencv-build EXCLUDE_FROM_ALL)
add_subdirectory(external/assimp-6.0.5 external/assimp-build EXCLUDE_FROM_ALL)

add_library(pthreadVC3 STATIC external/pthreads4w/pthread.c)
target_include_directories(pthreadVC3 PUBLIC external/pthreads4w)
target_compile_definitions(pthreadVC3 PUBLIC
    HAVE_CONFIG_H
    __PTW32_BUILD_INLINED
    __PTW32_CLEANUP_C
    __PTW32_STATIC_LIB
)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    target_compile_definitions(pthreadVC3 PUBLIC __PTW32_ARCHAMD64)
else()
    target_compile_definitions(pthreadVC3 PUBLIC __PTW32_ARCHX86)
endif()
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(pthreadVC3 PRIVATE -include stdint.h)
endif()

add_executable(ray_tracing_engine main.cpp)
target_link_libraries(ray_tracing_engine PRIVATE
    opencv_core
    opencv_imgcodecs
    assimp::assimp
    pthreadVC3
)
target_include_directories(ray_tracing_engine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/external/stb
)

if(MSVC)
    target_compile_options(ray_tracing_engine PRIVATE /W4)
else()
    target_compile_options(ray_tracing_engine PRIVATE -Wall -Wextra -Wpedantic)
endif()