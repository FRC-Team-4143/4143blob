cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(4143blob)
find_package(PCL 1.6 REQUIRED COMPONENTS common io filters segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS} "/usr/include/ni" include)
# /usr/include/ni is hack because package wasn't detected
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(4143blob src/4143blob.cpp include/4143blob.h)
target_link_libraries(4143blob ${PCL_LIBRARIES} )
