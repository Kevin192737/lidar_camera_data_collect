#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>

bool isWriten = false;

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
  if (isWriten) {
    ROS_INFO("pcl done");
    return;
  }
  ROS_INFO("pcl start");
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(input, cloud);//将PointCloud2转为PointXYZ
  pcl::io::savePCDFileASCII ("/home/hpf/ws1/test.pcd", cloud);    //保存pcd
  isWriten = true;
  ROS_INFO("pcl done");
}
int main (int argc, char **argv)
{
  ros::init (argc, argv, "pcl_writer");   
  ros::NodeHandle nh;
  ros::Subscriber bat_sub = nh.subscribe("/livox/lidar", 20, cloudCB);//接收点云
  // while (!isWriten)
  // {
  //   ros::Duration(0.1).sleep();
  //   ROS_INFO("Waiting...");
  // }
  ros::spin();
  
  return 0;
}