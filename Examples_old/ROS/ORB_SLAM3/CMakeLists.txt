cmake_minimum_required(VERSION 3.10.2)
project(orb_slam3_dense)

set( CMAKE_EXPORT_COMPILE_COMMANDS "ON" )

SET(CMAKE_BUILD_TYPE Debug)
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")  # todo 原 -03 改为 -O0

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -Wall -g -ggdb")#用于进行gdb调试,定义Debug编译参数 -g 表示包含调试信息
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall")# 定义Release编译参数

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(catkin REQUIRED COMPONENTS   # todo new
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        cv_bridge
        tf
        tf_conversions
        pcl_ros
        pcl_conversions
)

find_package(OpenCV 4 REQUIRED)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
   message(STATUS "OpenCV Find Version: ${OpenCV_VERSION}")
   message(STATUS "  OpenCV Include: ${OpenCV_INCLUDE_DIRS}")
   message(STATUS "OpenCV Libraries: ${OpenCV_LIBRARIES}")
endif()

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(eigen_conversions REQUIRED)



#find_package( PCL REQUIRED )  #原来：PCL 1.7.2 EXACT REQUIRED  # todo new
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)
message(STATUS "PCL library status:")
message(STATUS "    config: ${PCL_DIR}")
message(STATUS "    version: ${PCL_VERSION}")
message(STATUS "    libraries: ${PCL_LIBRARY_DIRS}")
message(STATUS "    include path: ${PCL_INCLUDE_DIRS}")

add_definitions( ${PCL_DEFINITIONS} )       # todo new
link_directories( ${PCL_LIBRARY_DIRS} )     # todo new

catkin_package(    # todo new
        INCLUDE_DIRS
        ${PROJECT_SOURCE_DIR}/../../../include
        #  LIBRARIES nav_goal
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        cv_bridge
        tf
        #  DEPENDS system_lib
)

include_directories(
${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include   # todo new
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${Pangolin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
${kdl_parser_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}# todo new
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${kdl_parser_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
        ${PCL_LIBRARIES}        # todo new
        ${catkin_LIBRARIES}     # todo new
        -lboost_system
)



# Node for rgbd-inertial-kinematic
add_executable(RGBD_Inertial_Kinematic
        src/ros_rgbd_inertial_kinematic.cc
        )

target_link_libraries(RGBD_Inertial_Kinematic
        ${LIBS}
        )
