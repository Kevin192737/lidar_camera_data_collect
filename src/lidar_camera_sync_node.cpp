#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <lidar_camera_sync/LidarCameraPointCloud.h>

ros::Publisher combined_pointcloud_pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg, const sensor_msgs::PointCloud2ConstPtr& camera_msg)
{
    // 创建自定义消息类型，用于存储合并后的数据
    lidar_camera_sync::LidarCameraPointCloud combined_msg;
    combined_msg.lidar_pointcloud = *lidar_msg;
    combined_msg.camera_pointcloud = *camera_msg;

    // 发布合并消息
    combined_pointcloud_pub.publish(combined_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_camera_pointcloud_publisher");
    ros::NodeHandle nh;

    // 创建发布器，发布合并后的消息
    combined_pointcloud_pub = nh.advertise<lidar_camera_sync::LidarCameraPointCloud>("combined_pointcloud", 1);

    // 创建订阅器，订阅雷达和相机的点云数据
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "lidar_pointcloud_topic", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> camera_sub(nh, "camera_pointcloud_topic", 1);

    // 使用时间同步策略来同步雷达和相机的点云数据
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), lidar_sub, camera_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}