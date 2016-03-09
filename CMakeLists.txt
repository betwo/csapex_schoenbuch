cmake_minimum_required(VERSION 2.8.3)
project(schoenbuch)


# Enforce that we use C++11
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#else()
#        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()



find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
#  csapex
#  csapex_point_cloud
#  csapex_core_plugins
#  csapex_ros
#  csapex_transform
)

find_package(PCL 1.7 REQUIRED COMPONENTS common filters segmentation)

#find_package(pcl_ros REQUIRED)

find_package(Eigen3 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
    DEPENDS Eigen3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(schoenbuch
    src/pillar_localization/ekf.cpp
    src/pillar_localization/pillar.cpp
    src/pillar_localization/pillar_extractor.cpp
)
target_link_libraries(schoenbuch
    #${pcl_ros_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${catkin_LIBRARIES}
)

#add_library(schoenbuch_node
#    src/pillar_localization.cpp
#)
#target_link_libraries(schoenbuch_node schoenbuch
#    ${catkin_LIBRARIES}
#)

add_executable(pillar_localization_node
    src/pillar_localization/pillar_localization_node.cpp
)
target_link_libraries(pillar_localization_node schoenbuch
  ${catkin_LIBRARIES}
)

