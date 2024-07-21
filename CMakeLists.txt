cmake_minimum_required(VERSION 2.8)
project(can)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.8 REQUIRED)
find_package(Threads REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(can main.cpp DBSCAN_simple.h DBSCAN_simple_quick.h  mine.h ransac.h file.h  nihe.h)
target_link_libraries (can ${PCL_LIBRARIES} Threads::Threads)

