cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project("automatic calibration")
find_package(PCL 1.3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++0x")

add_executable(
 bin/computecalibration
 src/computecalibration.cpp
 src/Common.h
 src/Common.cpp
 src/Visualization.h
 src/Visualization.cpp
 src/Triangulation.h
 src/Triangulation.cpp
)

target_link_libraries(bin/computecalibration 
			${OpenCV_LIBS}
			${PCL_LIBRARIES}
			)
