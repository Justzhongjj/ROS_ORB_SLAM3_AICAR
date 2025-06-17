/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>

#include"../../../include/System.h"


#include <geometry_msgs/PoseStamped.h> //modify
#include <tf/tf.h> 
#include <tf/transform_datatypes.h> 
#include"../../../include/Converter.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){
        //modify
         //创建ROS的发布节点     
	    //pub_tcw= nh.advertise<geometry_msgs::PoseStamped> ("/ORB_SLAM/pose", 10); 
	    pub_odom= nh.advertise<nav_msgs::Odometry> ("Odometry", 10); 
	    //pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 10);
        //cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 10);

        current_time = ros::Time::now();
        last_time = ros::Time::now();
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
    //modify
    ros::NodeHandle nh;
    ros::Publisher  pub_tcw,pub_camerapath,pub_odom;
    tf::TransformBroadcaster odom_broadcaster;
    nav_msgs::Path  camerapath;
    ros::Time current_time, last_time;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 10);

    //camera/color/image_raw
    //camera/aligned_depth_to_color/image_raw
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    //modify
    cv::Mat Tcw=cv::Mat(4, 4, CV_32F);
    bool  isKeyFrame =false;
    Sophus::SE3f se3=mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    Eigen::Matrix4f transform_matrix =se3.matrix();
     // 创建一个4x4的浮点型矩阵  
    for (int i = 0; i < 4; ++i) {  
        for (int j = 0; j < 4; ++j) {  
        Tcw.at<float>(i, j) = transform_matrix(i, j);  
        }  
    }

	current_time=ros::Time::now();
    
    if (!Tcw.empty()){
    /*******************************/

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
    
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);


    tf::Transform new_transform;
    new_transform.setOrigin(tf::Vector3( twc.at<float>(0), twc.at<float>(2),twc.at<float>(1)));
    tf::Quaternion quaternion;

    //交换y轴和z轴,先将四元素转矩阵，再获得欧拉角，再交换
    tf::Quaternion quat;
    geometry_msgs::PoseStamped robot_pose;
    tf::Matrix3x3 matrixw1;

    robot_pose.pose.orientation.x = q[0];
    robot_pose.pose.orientation.y = q[1];
    robot_pose.pose.orientation.z = q[2];
    robot_pose.pose.orientation.w = q[3];
    //相机位姿信息
    geometry_msgs::PoseStamped tcw_msg;
    tf::quaternionMsgToTF(robot_pose.pose.orientation, quat);
    double roll, pitch, yaw;//定义存储roll,pitch and yaw的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转化
    quaternion.setRPY(roll,yaw,-pitch);
    new_transform.setRotation(quaternion);
    tf::poseTFToMsg(new_transform, tcw_msg.pose);


    /*******************************/
    //publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x=tcw_msg.pose.position.x;
    odom_trans.transform.translation.y=tcw_msg.pose.position.y;
    odom_trans.transform.translation.z=0;
    odom_trans.transform.rotation= tcw_msg.pose.orientation;
    
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    /*******************************/

    
    /*******************************/
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom"; 
    tf::poseTFToMsg(new_transform, odom.pose.pose);
    //set the velocity 此时为假设数据,实际需要根据位姿计算
    //linear ：线速度  x:前 y:左 z:与x、y为右手螺旋法则关系
    //angular ：角速度 z:旋转轴
    odom.twist.twist.linear.x = 0.5;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.z = 0.3;

    /*
    odom_msg.pose.pose.position.x=tWC.at<float>(0);
    odom_msg.pose.pose.position.y=tWC.at<float>(1);			 
    odom_msg.pose.pose.position.z=tWC.at<float>(2);	
	
	odom_msg.pose.pose.orientation.x=q[0];
	odom_msg.pose.pose.orientation.y=q[1];
	odom_msg.pose.pose.orientation.z=q[2];
	odom_msg.pose.pose.orientation.w=q[3];
    */ 
    //publish the message  
	pub_odom.publish(odom);
    last_time=current_time;
    //Tcw位姿信息
	//pub_tcw.publish(tcw_msg);
    /*******************************/
    }
    else{
        ROS_INFO("Odometry is empty");
    }
}
