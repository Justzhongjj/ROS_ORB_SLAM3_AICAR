#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <iostream>
using namespace std;
 
ros::Publisher pub_point_cloud2;
 
bool is_K_empty = 1;
double K[9];
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]
 
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Step1: 读取深度图
    //ROS_INFO("image format: %s %dx%d", img_msg->encoding.c_str(), img_msg->height, img_msg->width);
    int height = img_msg->height;
    int width = img_msg->width;
    // 通过指针强制转换，读取为16UC1数据，单位是mm
    unsigned short *depth_data = (unsigned short*)&img_msg->data[0];
    
    // Step2: 深度图转点云
    sensor_msgs::PointCloud2 point_cloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int uy=0; uy<height; uy++)
    {
        for(int ux=0; ux<width; ux++)
        {
            float x, y, z;
            z = *(depth_data + uy*width + ux) / 1000.0;
            if(z>0.1 && z< 15)
            {
                x = -z * (ux - K[2]) / K[0];
                y = z * (uy - K[5]) / K[4];
                if(y>-1&&y<0.4){
                    pcl::PointXYZ p(x, y, z);
                    cloud->push_back(p);
                }
                
            }
        }
    }
    // Step3: 发布点云
    pcl::toROSMsg(*cloud, point_cloud2);
    point_cloud2.header.frame_id = "camera";
    pub_point_cloud2.publish(point_cloud2);
}
 
 
void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
{
    // 读取相机参数
    if(is_K_empty)
    {
        for(int i=0; i<9; i++)
        {
            K[i] = camera_info_msg->K[i];
        }
        is_K_empty = 0;
    }
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "piontcloud_pub");
    ros::NodeHandle n;
    // 订阅D435i的深度图，在其回调函数中把深度图转化为点云，并发布出来
    ros::Subscriber sub_img = n.subscribe("/camera/depth/image_rect_raw", 10, img_callback);
    // 订阅D435i的深度相机参数
    ros::Subscriber sub_cmara_info = n.subscribe("/camera/depth/camera_info", 1, camera_info_callback);
    pub_point_cloud2 = n.advertise<sensor_msgs::PointCloud2>("/Pointcloud2", 10);
    
    ROS_INFO("Runing ...");
    ros::spin();
    return 0;
}