#ifndef _Foothold_planner_H_
#define _Foothold_planner_H_

#include <thread>

// system information
#include <unistd.h>
#include <pwd.h>
#include <fstream>
#include <time.h>       /* time_t, struct tm, time, localtime, asctime */ //http://www.cplusplus.com/reference/ctime/asctime/

//! ROS
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

//! grid map
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_demos/FiltersDemo.hpp"

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_core/GridMap.hpp>

#include  "grid_map_core/GridMapMath.hpp"
#include  "grid_map_core/SubmapGeometry.hpp"
#include  "grid_map_core/iterators/GridMapIterator.hpp"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

// ROS
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>

// STL
#include <limits>
#include <algorithm>
#include <vector>
#include <math.h>
//pointcloud
#include <sensor_msgs/PointCloud.h>
//https://www.google.com.hk/search?q=github&oq=github&aqs=chrome..69i57j46i199i465i512j0i512l7.987j0j15&sourceid=chrome&ie=UTF-8>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//pcl point

#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>
// #include"T1/FootresultMsg.h"

//
#include <iostream>
#include <ctime>
#include <semaphore.h>//信号量需要的头文件

#include <sys/types.h>          
#include <sys/socket.h>
#include <netinet/in.h> 
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <arpa/inet.h>
#include<pthread.h>

#define IP "192.168.1.107"
#define CPORT 50001
//#define SPORT 50003
#define SPORT 9999

using namespace std;
using namespace grid_map;
namespace FootHoldPlanner{
class FootholdPlanner
{

    private:
         ros::Subscriber gridmapSub_;
         ros::Subscriber odom_Sub;
         ros::Subscriber chatter_Sub;
         ros::Publisher path_pub;
         nav_msgs::Path path;

       
    public:
        
        geometry_msgs::Point32 resultFootholdP;
        grid_map::Position default_foot;
        geometry_msgs::Point32 resultFoothold;

        geometry_msgs::PoseStamped this_pose_stamped;
        geometry_msgs::PointStamped defaultFoothold_;
      

        void foot_selection_part(GridMap gridMap, grid_map::Position p, grid_map::Position default_foot, grid_map::Length rect);
        void SubMsg(void);
        void FootholdPlannerInit(int argc, char **argv);
        void odom_Callback(const nav_msgs::Odometry::ConstPtr& odom);
        void gridmapCallback(grid_map_msgs::GridMap msg);
        void chatterCallback(const grid_map_msgs::GridMap& message);
       


};
/*-------------------------落足点规划检查--------------------*/
       

}
#endif