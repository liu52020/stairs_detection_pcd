cmake_minimum_required(VERSION 2.8.3)
project(stairs_detection)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(PCL 1.11 REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})


find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)


set (CMAKE_CXX_STANDARD 11)


include_directories(include)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(stairs src/main_stairs.cpp include/visualizer.h src/visualizer.cpp include/stair/current_scene_stair.h include/stair/stair_classes.h include/RGBD/current_scene.h src/RGBD/current_scene.cpp include/stair/global_scene_stair.h include/RGBD/global_scene.h include/RGBD/plane.h src/RGBD/global_scene.cpp src/RGBD/plane.cpp src/stair/current_scene_stair.cpp src/stair/stair_classes.cpp)


target_link_libraries(stairs
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
