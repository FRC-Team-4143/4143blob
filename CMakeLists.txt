cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(4143pclpyramid)
find_package(PCL 1.6 REQUIRED COMPONENTS common io filters segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS} "/usr/include/ni" include)
# /usr/include/ni is hack because package wasn't detected
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(4143pclpyramid src/4143pclpyramid.cpp include/4143pclpyramid.h)
target_link_libraries(4143pclpyramid ${PCL_LIBRARIES} )
