include(CMakeFindDependencyMacro)
find_dependency(OpenCV)
find_dependency(OpenMP)
include("${CMAKE_CURRENT_LIST_DIR}/FbowTargets.cmake")