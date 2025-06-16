#include <ros/ros.h>  
#include <sensor_msgs/PointCloud2.h>  
#include <sensor_msgs/LaserScan.h>  
#include <pcl_ros/transforms.h>  
#include <pcl/point_types.h>  
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>  
#include <limits>  
ros::Publisher pub;
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {  
    // 假设PointCloud2中的点是以米为单位的XYZ点  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::fromROSMsg(*msg, *cloud);  
    // 初始化LaserScan消息  
    sensor_msgs::LaserScan scan;  
    scan.header = msg->header; // 复制时间戳和坐标系  
    scan.header.frame_id = "lidar"; // 设置frame_id

    // 设置laserscan参数  
    scan.angle_min = 0;  //-M_PI / 2
    scan.angle_max = M_PI ;  //-M_PI / 2
    double angle_increment = 0.0087; // 根据需要设置角分辨率  
    int ranges_size = std::round((scan.angle_max - scan.angle_min) / angle_increment);  
    scan.angle_increment = angle_increment;  
    scan.ranges.resize(ranges_size, std::numeric_limits<float>::infinity());  
    scan.range_min = 0.1;  
    scan.range_max = 10.0;  
    
    // 遍历每个角度，并找到最近点的距离  
    for (int i = 0; i < ranges_size; ++i) {  
        double angle = scan.angle_min + i * scan.angle_increment;  
        double min_dist = std::numeric_limits<float>::infinity();  
        for (const auto& point : *cloud) {  
            double theta = atan2(point.y, point.x) - angle;  
            if (fabs(theta) < M_PI / 180.0 * 0.5) { // 假设激光在水平面附近，有一定的容差  
                double dist = sqrt(point.x * point.x + point.z * point.z);  
                if (dist < min_dist && dist > scan.range_min) {  
                    min_dist = dist;  
                }  
            }  
        }  
        if (min_dist != std::numeric_limits<float>::infinity()) {  
            scan.ranges[i] = min_dist;  
        }  
    }   
 

    ROS_INFO("laserscan publish...");
    pub.publish(scan);
    
}  
  
int main(int argc, char** argv) {  
    ros::init(argc, argv, "pointcloud_to_laserscan_node");  
    ros::NodeHandle nh;  
  
    // 订阅PointCloud2消息  
    ros::Subscriber sub = nh.subscribe("/Pointcloud2", 10, pointCloudCallback);    
    pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);  
    // 设置循环回调  
    ros::Rate rate(10); // 10Hz  
    ros::spin();
    return 0;  
}