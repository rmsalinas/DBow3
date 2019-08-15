cmake_minimum_required(VERSION 3.10)

find_package(OpenCV REQUIRED)

find_package(OpenMP  REQUIRED)

include(${CMAKE_SOURCE_DIR}/cmake/instructionSet.cmake)

if(BUILD_SHARED_LIBS)
    add_library(DBow3 SHARED)

    target_compile_definitions(DBow3 
                               PUBLIC 
                               DBOW_API
    )
    
else()
    add_library(DBow3 STATIC)
endif()

add_library(DBow3::DBow3 ALIAS DBow3)

target_sources(DBow3 
               PRIVATE 
               src/BowVector.cpp
               src/Database.cpp
               src/DescManip.cpp
               src/FeatureVector.cpp
               src/QueryResults.cpp
               src/ScoringObject.cpp
               src/Vocabulary.cpp
               src/quicklz.c
)

target_include_directories(DBow3
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
)

target_link_libraries(DBow3
                      PUBLIC
                      OpenMP::OpenMP_CXX
)

if(USE_OPENCV_CONTRIB)
target_link_libraries(DBow3
                      PUBLIC 
                      ${OpenCV_LIBS}
)

target_include_directories(DBow3
                           PUBLIC
                           ${OpenCV_INCLUDE_DIR}
)
endif()

if(USE_SIMD)
	target_link_libraries(DBow3
                        PRIVATE 
                        SIMD::SSE4.1
	)
endif()

target_compile_definitions(DBow3 
                           PUBLIC 
                           NOMINMAX 
                           _USE_MATH_DEFINES
)

target_compile_features(DBow3
                        PRIVATE 
                        cxx_std_14
)


install(TARGETS DBow3 EXPORT DBow3Targets
        LIBRARY  DESTINATION lib
        ARCHIVE  DESTINATION lib
        RUNTIME  DESTINATION bin
        INCLUDES DESTINATION include
)

install (DIRECTORY include/ DESTINATION include)

install(EXPORT DBow3Targets
        FILE DBow3Targets.cmake
        NAMESPACE DBow3::
        DESTINATION cmake/DBow3
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(DBow3ConfigVersion.cmake
	                             COMPATIBILITY SameMajorVersion
)

install(FILES DBow3Config.cmake ${CMAKE_CURRENT_BINARY_DIR}/DBow3ConfigVersion.cmake
        DESTINATION cmake/DBow3)