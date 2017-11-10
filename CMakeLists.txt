cmake_minimum_required(VERSION 2.8.3)
project(bino_stereo_ros)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	add_definitions(-DCOMPILEDWITHC11)
	message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	add_definitions(-DCOMPILEDWITHC0X)
	message(STATUS "Using flag -std=c++0x.")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(OpenCV 3.2.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS image_transport roscpp geometry_msgs std_msgs cv_bridge std_srvs sensor_msgs camera_info_manager)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
	)


#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
## Declare a cpp executable
add_executable(bino_stereo_ros src/bino_stereo.cpp)
target_link_libraries(bino_stereo_ros
	${PROJECT_SOURCE_DIR}/lib/libBinoCamera.so
	${catkin_LIBRARIES}
	${OpenCV_LIBS} 
	)
