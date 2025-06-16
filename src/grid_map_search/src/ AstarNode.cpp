#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "../include/grid_map_search/Astar.h"
#include <vector>
#include <ctime>

double origin_x=0.0;  //栅格坐标系与世界坐标系的原点X的差值，单位为m
double origin_y=0.0;  //栅格坐标系与世界坐标系的原点Y的差值，单位为m
double width=0.0;     //栅格地图的宽
double height=0.0;    //栅格地图的高
double resolution=0.0;//栅格地图的分辨率

std::vector<int>mapData;
std::vector<std::vector<int> > startMapPoint; //Astar算法的起点xy值
std::vector<std::vector<int> > goalMapPoint;  //Astar算法的起点xy值
std::vector<std::pair<int, int> >astarPath;   //Astar算法的路径点

ros::Publisher astarPathPub;      //发布Astar路径
ros::Publisher astarStartPoint;   //发布起点位置
ros::Publisher astarGoalPoint;    //发布终点位置
ros::Subscriber mapSub;           //订阅栅格地图
ros::Subscriber initPoseSub;      //订阅起点
ros::Subscriber goalPosePub;      //订阅终点

std::string distance;            //Astar的距离函数
double weight_a;                 //Astar的权重a值
double weight_b;                 //Astar的权重b值

void PublishPath();
std::vector<int> WorldToMap(double wx,double wy);
std::vector<double>MapToWorld(double my,double mx);

void StartFindPath()
{ 
 
    //获得栅格坐标系下的起点 
   // int xStart=startMapPoint[0][0];
    //int yStart=startMapPoint[0][1];
    //std::cout<<"start:"<<xStart<<","<<yStart<<std::endl;
    const std::vector<int>& last_startMapPoint=startMapPoint.back();
    int xStart=last_startMapPoint[0];
    int yStart=last_startMapPoint[1];
    std::cout<<"start:"<<xStart<<","<<yStart<<std::endl;

    //获得栅格坐标系下的终点
    //int xStop=goalMapPoint[0][0];
   // int yStop=goalMapPoint[0][1];
    //std::cout<<"goal:"<<xStop<<","<<yStop<<std::endl;

    const std::vector<int>& last_goalMapPoint=goalMapPoint.back();
    int xStop=last_goalMapPoint[0];
    int yStop=last_goalMapPoint[1];
    std::cout<<"goal:"<<xStop<<","<<yStop<<std::endl;

    std::cout<<"\033[0;32m[I] : Start find path with Astar \033[0m"<<std::endl;
    //构建Astar的对象
    ASTAR::CAstar astar(xStart, yStart, xStop, yStop, weight_a, weight_b,ASTAR::CAstar::PathType::NOFINDPATHPOINT,distance);
    astar.InitMap(mapData,width,height);

    clock_t time_astar=clock();
    astarPath = astar.PathPoint(); //Astar算法的核心函数
    std::cout<<"\033[0;31m[I] Astar_time:\033[0m"<<1000*(clock()-time_astar)/(double)CLOCKS_PER_SEC<<"ms"<<std::endl;
    std::cout<<"Astar size:"<<astarPath.size()<<std::endl;

    if (astar.m_noPathFlag == ASTAR::CAstar::PathType::NOFINDPATHPOINT)
    {
        std::cout << "A星算法没能找到路径!!!" << std::endl;
    }
    else
    {
        PublishPath();
    } 
}

//世界-->地图坐标系
std::vector<int> WorldToMap(double wx,double wy)
{
    std::vector<int> v;
    if (wx<(1.0*origin_x) || wy<(1.0*origin_y))
    {
        v.push_back(-1);
        v.push_back(-1);
        return v;
    }
    int mx=int((1.0*(wx-origin_x))/resolution);
    int my=int((1.0*(wy-origin_y))/resolution);
    if (mx<width && my<height)
    {
        v.push_back(my);
        v.push_back(mx);
        return v;
    }
}

//地图-->世界坐标系
std::vector<double>MapToWorld(double my,double mx)
{
   std::vector<double> v;
   if(mx>width || my>height)
   {
       v.push_back(-1);
       v.push_back(-1);
       return v;
   }
   double wx=(mx*resolution+origin_x);
   double wy=(my*resolution+origin_y);
   if (wx>origin_x&&wy>origin_y)
   {
       v.push_back(wx);
       v.push_back(wy);
       return v;
   }
}

//栅格地图的回调函数
void MapCallback( const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    origin_x=msg->info.origin.position.x; //获得栅格地图的原点x值(相对世界坐标系),单位为m
    origin_y=msg->info.origin.position.y; //获得栅格地图的原点y值(相对世界坐标系),单位为m
    resolution=msg->info.resolution; //获得栅格地图的分辨率
    width=msg->info.width;     //获得栅格地图的宽
    height=msg->info.height;   //获得栅格地图的高
    std::cout<<"***********map message**********"<<std::endl;
    std::cout<<"origin_x:"<<origin_x<<std::endl;
    std::cout<<"origin_y:"<<origin_y<<std::endl;
    std::cout<<"resolution:"<<resolution<<std::endl;
    std::cout<<"width:"<<width<<std::endl;
    std::cout<<"height:"<<height<<std::endl;
    std::cout<<"*********************************"<<std::endl;
    mapData.resize(width*height);
    //获得地图数据 0自由通行区域，100障碍物
    for (int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            mapData[i*width+j]=int (msg->data[i*width+j]);
        }
    }
}

//初始位置的回调函数
void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{  
    //rviz中可视化的起始点
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;
    

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 1.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = resolution*2;
    node_vis.scale.y = resolution*2;
    node_vis.scale.z = resolution*2;
    
    geometry_msgs::Point pt;
    pt.x=msg->pose.pose.position.x;
    pt.y=msg->pose.pose.position.y;
    pt.z=0.0;
    node_vis.points.push_back(pt);
    astarStartPoint.publish(node_vis);
    startMapPoint.push_back(WorldToMap(msg->pose.pose.position.x,msg->pose.pose.position.y));
    
    if (startMapPoint[0][0]==-1&&startMapPoint[0][1]==-1)
        std::cout<<"\033[0;31m[E] : Please set the valid goal point\033[0m"<<std::endl;
}

//终点位置的回调函数
void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;
    
    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;
    node_vis.scale.x = resolution*2;
    node_vis.scale.y = resolution*2;
    node_vis.scale.z = resolution*2;

    geometry_msgs::Point pt;
    pt.x=msg->pose.position.x;
    pt.y=msg->pose.position.y;
    pt.z=0.0;
    node_vis.points.push_back(pt); 
    astarGoalPoint.publish(node_vis);
    goalMapPoint.push_back(WorldToMap(msg->pose.position.x,msg->pose.position.y));
    
    if (goalMapPoint[0][0]==-1&&goalMapPoint[0][1]==-1)
        std::cout<<"\033[0;30m[Kamerider E] : Please set the valid goal point\033[0m"<<std::endl;
    else
        StartFindPath();
}

//发布Astar路径轨迹
void PublishPath()
{
    nav_msgs::Path astarPathTopic;    //Astar路径的话题名
    for(int i=0;i<astarPath.size();i++)
    {   
        geometry_msgs::PoseStamped pathPose;
        pathPose.pose.position.x=MapToWorld(astarPath[i].first,astarPath[i].second)[0];
        pathPose.pose.position.y=MapToWorld(astarPath[i].first,astarPath[i].second)[1];
        pathPose.pose.position.z=0;

        pathPose.pose.orientation.x = 0.0;
        pathPose.pose.orientation.y = 0.0;
        pathPose.pose.orientation.z = 0.0;
        pathPose.pose.orientation.w = 1.0;
        astarPathTopic.header.stamp=ros::Time::now();
        astarPathTopic.header.frame_id="odom";
        astarPathTopic.poses.push_back(pathPose); 
    }   
    astarPathPub.publish(astarPathTopic);
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"astar_node");
    ros::NodeHandle nh("~");

    astarPathPub=nh.advertise<nav_msgs::Path>("path_my_Astar",15); //发布Astar路径
    mapSub=nh.subscribe("/map",1,MapCallback);  //订阅栅格地图
    initPoseSub=nh.subscribe("/initialpose",1,InitPoseCallback); //订阅起点
    goalPosePub=nh.subscribe("/move_base_simple/goal",1,GoalPoseCallback);//订阅终点
    astarStartPoint = nh.advertise<visualization_msgs::Marker>("/start_point", 10); //可视化起点位置
    astarGoalPoint = nh.advertise<visualization_msgs::Marker>("/goal_point", 10);  //可视化终点位置

    nh.param("heuristic/distance",distance,std::string("euclidean")); //距离参数
    nh.param("weight/a",weight_a,1.0);  //权重a值
    nh.param("weight/b",weight_b,1.0);  //权重b值

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      ros::spinOnce();      
      loop_rate.sleep();
    }
    return 0;
}
