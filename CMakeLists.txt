cmake_minimum_required(VERSION 3.10)

project(DBow3 VERSION 1.0.0)

option(BUILD_EXAMPLES "Set to ON to build examples" OFF)
option(BUILD_TESTS "Set to ON to build tests" OFF)
option(BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON)

# OpenCV
option(USE_OPENCV_CONTRIB "Set on to use opencv-contrib" ON)

# Optimization
option(USE_SIMD "Set on to enabel avx optimization" ON)


add_subdirectory(Lib)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(BUILD_EXAMPLES)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif(BUILD_TESTS)

