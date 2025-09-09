cmake_minimum_required(VERSION 2.8.3)
project(da_lio)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++17 )
ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions  
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED timer) #GTSAM require
find_package(GTSAM REQUIRED PATHS "/home/third_library/gtsam-4.0.3/install")
find_package(Ceres REQUIRED PATHS "/home/third_library/ceres-solver-2.1.0/install")

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_library(basic_odom include/registration.cpp include/common_pt_operations.cpp include/io.cpp)
target_link_libraries(basic_odom ${catkin_LIBRARIES} gtsam tbb ${PCL_LIBRARIES} -lstdc++fs ${CERES_LIBRARIES} )

add_executable(da_lio src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_link_libraries(da_lio ${PYTHON_LIBRARIES} basic_odom)
target_include_directories(da_lio PRIVATE ${PYTHON_INCLUDE_DIRS})
