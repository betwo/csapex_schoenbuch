cmake_minimum_required(VERSION 2.8.3)
project(pillar_localization)

# Enforce that we use C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    add_definitions(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
    add_definitions(-std=c++0x)
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  csapex csapex_point_cloud csapex_transform csapex_core_plugins csapex_ros
)
find_package(PCL 1.7 REQUIRED COMPONENTS common filters segmentation)

find_package(Eigen3 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    DEPENDS Eigen3
)

include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# headers
#file(GLOB_RECURSE files "include/*")
#add_custom_target(headers SOURCES ${files})

add_library(schoenbuch
    src/data/point.cpp
    src/data/cluster.cpp

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
    src/object_detector.cpp
    src/person_detector.cpp
    src/param_estimator.cpp
    src/pose_difference.cpp
    src/imu_integration.cpp
    src/velodyne_undistortion.cpp
    src/loam_segmentation.cpp
    src/velodyne_segmentation.cpp
    src/object_tracker.cpp

    src/local_scrolling_map.cpp
    src/local_hazard_field.cpp
)
target_link_libraries(schoenbuch_node schoenbuch
    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
    ${csapex_point_cloud_LIBRARIES}
    ${csapex_core_plugins_LIBRARIES}
    ${csapex_transform_LIBRARIES}
    ${csapex_ros_LIBRARIES}
)

add_executable(pillar_localization_node
    src/pillar_localization/pillar_localization_node.cpp
)
target_link_libraries(pillar_localization_node schoenbuch
    ${catkin_LIBRARIES}
)


install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS schoenbuch_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS schoenbuch
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS pillar_localization_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

