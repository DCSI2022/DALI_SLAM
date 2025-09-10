cmake_minimum_required(VERSION 3.0.2)
project(backend_mapping)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

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

find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED) # ikd-tree require
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
find_package(Boost REQUIRED timer) #GTSAM require
find_package(GTSAM REQUIRED PATHS "/home/third_library/gtsam-4.0.3/install")
find_package(Ceres REQUIRED PATHS "/home/third_library/ceres-solver-2.1.0/install")
find_package(PDAL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        rosbag
        roscpp
        livox_ros_driver
        pcl_ros
        )

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
)

add_library(basic include/io.cpp include/ikd-Tree/ikd_Tree.cpp
        include/imu_process.cpp include/registration.cpp include/common_pt_operations.cpp)
target_link_libraries(basic ${catkin_LIBRARIES} gtsam tbb ${PCL_LIBRARIES} -lstdc++fs ${CERES_LIBRARIES} pdalcpp)

add_library(fast_gicp
        include/fast_gicp/lsq_registration.cpp
        include/fast_gicp/fast_gicp.cpp
        include/fast_gicp/fast_vgicp.cpp
        )
target_link_libraries(fast_gicp
        ${PCL_LIBRARIES})

add_executable(mc_pgo src/pose_graph_opti.cpp)
target_link_libraries(mc_pgo basic fast_gicp)

add_executable(pgo_base src/pose_graph_opti_base.cpp)
target_link_libraries(pgo_base basic)

add_executable(map_mme src/mean_map_entropy.cpp)
target_link_libraries(map_mme basic)

add_executable(test_fastgicp src/test_fastgicp.cpp)
target_link_libraries(test_fastgicp fast_gicp)