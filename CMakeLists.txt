cmake_minimum_required(VERSION 2.8.3)
project(lviorf)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  roslib
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
  lib
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
  lib
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########
# <!-- lviorf_yjz_lucky_boy -->
file(GLOB common_lib "lib/*.cpp")

# LIO
# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/lio/imageProjection.cpp ${common_lib})
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/lio/mapOptmization.cpp ${common_lib})
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/lio/imuPreintegration.cpp ${common_lib})
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)


######################
# <!-- lviorf_yjz_lucky_boy -->
# VIO
######################
file(GLOB visual_feature_files
    "src/vio/visual_feature/*.cpp"
    "src/vio/visual_feature/camera_models/*.cc"
)
file(GLOB visual_odometry_files
    "src/vio/visual_estimator/*.cpp"
    "src/vio/visual_estimator/factor/*.cpp"
    "src/vio/visual_estimator/initial/*.cpp"
    "src/vio/visual_estimator/utility/*.cpp"
)
file(GLOB visual_loop_files
    "src/vio/visual_loop/*.cpp"
    "src/vio/visual_loop/utility/*.cpp"
    "src/vio/visual_loop/ThirdParty/*.cpp"
    "src/vio/visual_loop/ThirdParty/DBoW/*.cpp"
    "src/vio/visual_loop/ThirdParty/DUtils/*.cpp"
    "src/vio/visual_loop/ThirdParty/DVision/*.cpp"
    "src/vio/visual_feature/camera_models/*.cc"
)
# Visual Feature Tracker
add_executable(${PROJECT_NAME}_visual_feature ${visual_feature_files})
target_link_libraries(${PROJECT_NAME}_visual_feature ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
# Visual Odometry
add_executable(${PROJECT_NAME}_visual_odometry ${visual_odometry_files})
target_link_libraries(${PROJECT_NAME}_visual_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
# Visual Lopp
add_executable(${PROJECT_NAME}_visual_loop ${visual_loop_files})
target_link_libraries(${PROJECT_NAME}_visual_loop ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 