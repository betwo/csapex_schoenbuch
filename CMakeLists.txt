cmake_minimum_required(VERSION 2.8.3)
project(schoenbuch)

find_package(catkin REQUIRED COMPONENTS
  csapex
  csapex_point_cloud
  csapex_core_plugins
  csapex_transform
)

find_package(PCL 1.2 REQUIRED COMPONENTS common)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(schoenbuch_node
    src/pillar_localization.cpp
)

target_link_libraries(schoenbuch_node
  ${catkin_LIBRARIES}
)
