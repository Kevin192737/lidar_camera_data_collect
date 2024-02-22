cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_sync)

## 找到 catkin 宏和库
## 维护 catkin 包和工作区的配置信息
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  message_generation
  pcl_conversions
  cv_bridge
  tf2_ros
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
## 声明msg文件,并传递给 generate_messages() 函数
add_message_files(
  FILES
  LidarCameraPointCloud.msg
)

## 在 generate_messages() 函数中添加消息依赖项
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

## 声明本包是一个 catkin 包，还需要声明对其他 catkin 包的依赖关系
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros message_runtime pcl_conversions cv_bridge
)

## 指定包含的头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
)

## 创建可执行文件
add_executable(lidar_camera_sync_node src/lidar_camera_sync_node.cpp)

## 链接可执行文件到依赖的库
target_link_libraries(lidar_camera_sync_node
  ${catkin_LIBRARIES})  

## 创建可执行文件
add_executable(pcl_writer src/pcl_write.cpp)

## 链接可执行文件到依赖的库
target_link_libraries(pcl_writer 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

  ## 创建可执行文件
add_executable(image_pointcloud_sync_node src/image_pointcloud_sync.cpp)

## 链接可执行文件到依赖的库
target_link_libraries(image_pointcloud_sync_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES})