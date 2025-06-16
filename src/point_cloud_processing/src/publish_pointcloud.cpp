#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

using namespace std;

int main (int argc, char **argv)  
{  
	std::string topic,path,frame_id;
    int hz=5;

	ros::init (argc, argv, "publish_pointcloud");  
	ros::NodeHandle nh;  
	// /home/zjj/data/map/pointcloud/map1.pcd/
    nh.param<std::string>("path", path, "/home/zjj/data/map/pointcloud/407map4.pcd");//
	nh.param<std::string>("frame_id", frame_id, "map");
	nh.param<std::string>("topic", topic, "/pointcloud/output");
    nh.param<int>("hz", hz, 5);
   
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> (topic, 10);  

	pcl::PointCloud<pcl::PointXYZ> cloud;  
	sensor_msgs::PointCloud2 output;  
	
	 
	while (ros::ok())  
	{  
		pcl::io::loadPCDFile (path, cloud);  
		//旋转点云
    	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    	transform.rotate(Eigen::AngleAxisf(-M_PI/2,Eigen::Vector3f(1,0,0)));     //-M_PI*5/12
    	pcl::transformPointCloud(cloud,cloud,transform);
		pcl::toROSMsg(cloud,output);// 转换成ROS下的数据类型 最终通过topic发布
		output.header.stamp=ros::Time::now();
		output.header.frame_id  =frame_id;
		cout<<"path = "<<path<<endl;
		cout<<"frame_id = "<<frame_id<<endl;
		cout<<"topic = "<<topic<<endl;
		cout<<"hz = "<<hz<<endl;
		ros::Rate loop_rate(hz); 
		cout<<"output pointclud height:"<<output.height<<endl;
        cout<<"output pointcloud width:"<<output.width<<endl<<endl;
		pcl_pub.publish(output);  
		ros::spinOnce();  
		loop_rate.sleep();  
	}  
	return 0;  
} 
