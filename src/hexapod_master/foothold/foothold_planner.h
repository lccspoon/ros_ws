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
//#include <grid_map_msgs/GridMap.h>
//#include <grid_map_ros/grid_map_ros.hpp>
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

namespace FootHoldPlanner{
class FootholdPlanner
{

    private:
         ros::Subscriber gridmapSub_;
         ros::Subscriber odom_Sub;
         ros::Publisher path_pub;
         nav_msgs::Path path;

       
    public:
        
        float candidateFootholdThreshold_=0.7;
        float defaultFootholdThreshold_=0.7;

        grid_map::GridMap gridmap_; // 全局地图
        grid_map::GridMap gaitMap_; // 子地图，一个步态周期默认等腰梯形对应的gridmap


        geometry_msgs::PoseStamped this_pose_stamped;
        geometry_msgs::PointStamped defaultFoothold_;
        // void gridmapCallback(grid_map_msgs::GridMap msg);
        std::string mapFrame_="map";

        double searchRadius_=1;

        void SubMsg(void);
        void FootholdPlannerInit(int argc, char **argv);
        void odom_Callback(const nav_msgs::Odometry::ConstPtr& odom);
        void gridmapCallback(grid_map_msgs::GridMap msg);


    /********************************************
     * 获取gridmap地图指定坐标位置对应的索引值
     * 输入：gridmap
     *      坐标位置
     * 输出：索引值
     * 返回：成功标志
    *******************************************/
    bool getMapIndex(   grid_map::GridMap map,
                        geometry_msgs::Point position,                                                
                        grid_map::Index& index);

    /********************************************
        检查同时落在设定的圆形及矩形区域的落脚点的可通行性
        输入：grid map；
             圆形区域中心；
             圆形区域半径；
             四边形区域；
        返回：是否可通行
    *******************************************/
    bool checkCirclePolygonFoothold(grid_map::GridMap gridmap, 
                                    grid_map::Position center,
                                    float searchRadius,
                                    grid_map::Polygon polygon);

    /********************************************
        获取落脚点搜索矩形区域
        输入：圆形搜索区域半径；
             圆形搜索区域中心
        返回：矩形搜索区域
    *******************************************/
    grid_map::Polygon getSearchPolygon( geometry_msgs::PointStamped center,
                                        float radius);

    geometry_msgs::PolygonStamped getSearchPolygon( geometry_msgs::Vector3 center,
                                                    float radius);     


        struct gridMapInfo_
    {
        double x;
        double y;
        double length; // xss
        double width; // y
        int row; // x
        int col; // y
        double resolution;
        std::string frameID;
    }gridMapInfo_;

        bool checkFootholdUseCentroidMethod(grid_map::GridMap& gridmap,
                                        geometry_msgs::PointStamped defaultFoothold);
        // bool readParameters();



};
/*-------------------------落足点规划检查--------------------*/
       

}
#endif