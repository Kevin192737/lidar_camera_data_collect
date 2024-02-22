#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::Pose get_relative_pose(const std::string base, const std::string target)
{
geometry_msgs::Pose pose_msg;
   static tf2_ros::Buffer buffer;

  static tf2_ros::TransformListener listener(buffer);
  std::string err;

  while (!buffer.canTransform(base, target, ros::Time(0), &err))

  {}

  auto ts = buffer.lookupTransform(base, target, ros::Time(0));

  geometry_msgs::Pose pose;

  pose.position.x = ts.transform.translation.x;

  pose.position.y = ts.transform.translation.y;

  pose.position.z = ts.transform.translation.z;

  pose.orientation = ts.transform.rotation;

  return pose;
}

void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg)
{
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 保存图像
    std::string image_filename = "/home/hpf/ws1/src/lidar_camera_sync/data/image/image" + std::to_string(image_msg->header.stamp.toSec()) + ".jpg";
    cv::imwrite(image_filename, cv_ptr->image);

    // 保存点云数据
    std::string pointcloud_filename = "/home/hpf/ws1/src/lidar_camera_sync/data/lidar/lidar" + std::to_string(pointcloud_msg->header.stamp.toSec()) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*pointcloud_msg, cloud);//将PointCloud2转为PointXYZ

    Eigen::Affine3f transform=Eigen::Affine3f::Identity();
    // transform.translation()<<-0.0515101,
    //                         0.0268540,
    //                         0.0588544;
    // Eigen::Quaternionf q(0.50107211,
    //                     0.4898137,
    //                     0.50072068,
    //                     0.5082205);
    geometry_msgs::Pose pose = get_relative_pose("camera_color_optical_frame","livox_frame");
    std::cout << "####Pose:\n" << pose << std::endl;
    transform.translation().x() = pose.position.x;
    transform.translation().y() = pose.position.y;
    transform.translation().z() = pose.position.z;
    Eigen::Quaternionf q(pose.orientation.w,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z);
    transform.rotate(q);
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(cloud,transformed_cloud,transform);
    pcl::io::savePCDFileASCII (pointcloud_filename, transformed_cloud);    //保存pcd
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_pointcloud_sync_node");
    ros::NodeHandle nh;

    // 创建订阅器，分别订阅相机图像和雷达点云数据
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/livox/lidar", 1);

    // 使用时间同步器将相机图像和雷达点云数据进行时间同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, pointcloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}