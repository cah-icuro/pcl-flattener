cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(flatten_pcl)

# std=c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# PCL Library
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Build executable
add_executable(../new_flatten_pcl flatten_pcl_new.cpp util.cpp)

# Link PCL libraries - must come after the executable line
target_link_libraries(../new_flatten_pcl ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

