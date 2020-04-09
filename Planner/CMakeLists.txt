cmake_minimum_required(VERSION 2.8)
project(pointcloudTraj)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14 -march=native -DEIGEN_DONT_PARALLELIZE ${CMAKE_CXX_FLAGS} -g -O0 -Wall")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        sensor_msgs
        quadrotor_msgs
        tf
        kdtree)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 EXACT REQUIRED)

catkin_package(
        INCLUDE_DIRS include)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS})

link_directories(
        ${PCL_LIBRARY_DIRS}
        ${PROJECT_SOURCE_DIR}/lib/mosek7)

add_executable(sim_planning_demo
        src/sim_planning_demo.cpp
        src/traj_optimizer.cpp
        src/corridor_finder.cpp
        src/bezier_base.cpp)

target_link_libraries(sim_planning_demo
        mosek64
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})

add_executable(odom_generator
        src/odom_generator.cpp)

target_link_libraries(odom_generator
        ${catkin_LIBRARIES})

add_executable(traj_server
        src/traj_server.cpp
        src/bezier_base.cpp)

target_link_libraries(traj_server
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})

add_executable(camera_sensor
        src/camera_sensor.cpp
        src/map_observer.cpp
        src/cone_keeper.cpp
        src/utils.cpp)

target_link_libraries(camera_sensor
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})

add_executable(map_provider
        src/map_provider.cpp
        src/map_generator.cpp
        src/voxel_map.cpp)

target_link_libraries(map_provider
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})

add_executable(map_rendering_test
        src/map_rendering_test.cpp
        src/map_generator.cpp
        src/voxel_map.cpp
        src/map_observer.cpp)

target_link_libraries(map_rendering_test
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})

add_executable(map_exploring_test
        src/map_exploring_test.cpp
        src/map_generator.cpp
        src/cone_keeper.cpp
        src/utils.cpp
        src/voxel_map.cpp
        src/map_observer.cpp)

target_link_libraries(map_exploring_test
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})

add_executable(trajectory_keeper
        src/trajectory_keeper.cpp)

target_link_libraries(trajectory_keeper
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})

add_executable(status_inspector
        src/status_inspector.cpp)

target_link_libraries(status_inspector
        ${catkin_LIBRARIES})

add_executable(trajectories_comparison
        src/trajectories_comparison.cpp)

target_link_libraries(trajectories_comparison
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})