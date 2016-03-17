cmake_minimum_required(VERSION 2.8.3)
project(pillar_localization)


# Enforce that we use C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
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
)

find_package(csapex REQUIRED)
find_package(csapex_point_cloud REQUIRED)
find_package(csapex_core_plugins REQUIRED)
find_package(csapex_transform REQUIRED)
find_package(csapex_ros REQUIRED)

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
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${csapex_INCLUDE_DIRS}
  ${csapex_point_cloud_INCLUDE_DIRS}
  ${csapex_core_plugins_INCLUDE_DIRS}
  ${csapex_transform_INCLUDE_DIRS}
  ${csapex_ros_INCLUDE_DIRS}
)

# headers
file(GLOB_RECURSE files "include/*")
add_custom_target(headers SOURCES ${files})

# don't use c++ 11 -> segfaults...
unset(CMAKE_CXX_FLAGS)

add_library(schoenbuch
    src/pillar_localization/pillar.cpp
    src/pillar_localization/pillar_extractor.cpp
    src/pillar_localization/pillar_localization.cpp

    src/ekf/ekf.cpp
)
target_link_libraries(schoenbuch
    ${PCL_COMMON_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(schoenbuch_node
    src/pillar_localization.cpp
    src/param_estimator.cpp
    src/pose_difference.cpp
)
target_link_libraries(schoenbuch_node schoenbuch
    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
    ${csapex_point_cloud_LIBRARIES}
    ${csapex_core_plugins_LIBRARIES}
    ${csapex_transform_LIBRARIES}
    ${csapex_ros_LIBRARIES}
)

if(COMPILER_SUPPORTS_CXX11)
    SET_TARGET_PROPERTIES(schoenbuch_node PROPERTIES COMPILE_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    SET_TARGET_PROPERTIES(schoenbuch_node PROPERTIES COMPILE_FLAGS "-std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_executable(pillar_localization_node
    src/pillar_localization/pillar_localization_node.cpp
)
target_link_libraries(pillar_localization_node schoenbuch
    ${catkin_LIBRARIES}
)

