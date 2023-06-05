#include"foothold_planner.h"
// #include"ros/ros.h"
using namespace std;
using namespace FootHoldPlanner;
double mapResolution = 0.02; // gridmap分辨率

geometry_msgs::PointStamped P1;

/*----------------------回调函数-------------------------*/
//Traversability 回调函数  
void FootholdPlanner::gridmapCallback(grid_map_msgs::GridMap msg){

    // grid_map::GridMap gridmap_; // 全局地图
    // // grid_map::GridMap gaitMap_; // 子地图，一个步态周期默认等腰梯形对应的gridmap

    grid_map::GridMapRosConverter::fromMessage(msg, gridmap_);
    gridMapInfo_.x = gridmap_.getPosition()(0);
    gridMapInfo_.y = gridmap_.getPosition()(1);
    gridMapInfo_.length = gridmap_.getLength().x();
    gridMapInfo_.width = gridmap_.getLength().y();
    gridMapInfo_.row = gridmap_.getSize()(0);
    gridMapInfo_.col = gridmap_.getSize()(1);
    mapResolution = gridMapInfo_.resolution = gridmap_.getResolution();
    gridMapInfo_.frameID = gridmap_.getFrameId();    

    cout<<endl<<"Traversability GridMap info: "<<endl;
    printf("  size = %f x %f m (%i x %i cells)\n  resolution = %f m/cell\n  position = (%f, %f)m\n  frame = %s\n", 
        gridMapInfo_.length, gridMapInfo_.width,
        gridMapInfo_.row, gridMapInfo_.col,
        gridMapInfo_.resolution,
        gridMapInfo_.x, gridMapInfo_.y,
        gridMapInfo_.frameID.c_str());
    cout<<endl;

   bool validation;
//    validation=validation = checkCirclePolygonFoothold(gridmap, footCenter, footRadius, polygon);
    validation=checkFootholdUseCentroidMethod(gridmap_,defaultFoothold_);
}
//里程计回调函数
void FootholdPlanner::odom_Callback(const nav_msgs::Odometry::ConstPtr& odom)
{
     clock_t odom_startTime, odom_endTime;
     odom_startTime = clock();
    //geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom -> pose.pose.position.x;
    this_pose_stamped.pose.position.y = odom -> pose.pose.position.y;
    this_pose_stamped.pose.position.z = odom -> pose.pose.position.z;
 
    this_pose_stamped.pose.orientation = odom -> pose.pose.orientation;
 
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "base_link";

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    // path_pub.publish(path);
    odom_endTime = clock();
    // cout<<"the running time of odom_callback"<<(double)(odom_startTime - odom_endTime) / CLOCKS_PER_SEC <<'s'<<endl;
    // printf("path_pub:");
    // printf("odom %.3lf %.3lf %.3lf\n", odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);

    defaultFoothold_.point.x=this_pose_stamped.pose.position.x;
    defaultFoothold_.point.y=this_pose_stamped.pose.position.y;
    defaultFoothold_.point.z=this_pose_stamped.pose.position.z;

    // cout <<"defaultFoothold_"<<defaultFoothold_.point.x << endl;

 
    // P1.point.x=this_pose_stamped.pose.position.x+0.25;
    // P1.point.y=this_pose_stamped.pose.position.y;

    // defaultFoothold_.point.x=P1.point.x;
    // defaultFoothold_.point.x=P1.point.y;
}
/*-----------------------类里面的函数定义-----------------*/
//初始化函数
void FootholdPlanner::FootholdPlannerInit(int argc, char **argv)
{
    ros::init(argc,argv,"map");
    ros::NodeHandle n;
}
//订阅函数
void FootholdPlanner::SubMsg(void)
{
    ros::NodeHandle m;
    odom_Sub = m.subscribe<nav_msgs::Odometry>("/odom_now", 100, &FootholdPlanner::odom_Callback,this);
    gridmapSub_ = m.subscribe("/traversability_estimation/traversability_map", 100, &FootholdPlanner::gridmapCallback,this);
    // odom_Sub = m.subscribe<nav_msgs::Odometry>("/odom_now", 10, &FootholdPlanner::odom_Callback,this);
 

//
}
//
bool FootholdPlanner::getMapIndex(  grid_map::GridMap map,
                                    geometry_msgs::Point position,                                                
                                    grid_map::Index& index)
{
    grid_map::Position p;
    p.x() = position.x;
    p.y() = position.y;

    if(!map.getIndex(p, index)) return false;

    return true;
}
//
bool FootholdPlanner::checkCirclePolygonFoothold(   grid_map::GridMap gridmap, 
                                                    grid_map::Position center,
                                                    float footRadius,
                                                    grid_map::Polygon polygon){
//   if(checkDefaultFoothold_Debug_) ROS_INFO("Checking circle and polygon foothold validation ......");

  bool validation = false;  
  bool isNAN = false;
      
  for(grid_map::CircleIterator iterator(gridmap, center, footRadius); !iterator.isPastEnd(); ++iterator)
  {
    // if(checkDefaultFoothold_Debug_){
        ROS_INFO_STREAM( "Value at index " << (*iterator).transpose() << " = " << gridmap.at("traversability", *iterator ) );
    // }      

    if(gridmap.isValid((*iterator).transpose(), "traversability"))
    {
        grid_map::Index index((*iterator).transpose());
        grid_map::Position p;
        gridmap.getPosition(index, p);      

        if(gridmap.at("traversability", *iterator) < candidateFootholdThreshold_ || false == polygon.isInside(p)) 
        {
            validation = false;
            isNAN = true;

            // if(checkDefaultFoothold_Debug_){
                ROS_INFO_STREAM( "Value at index " << (*iterator).transpose() << " = " << gridmap.at("traversability", *iterator ) );
                ROS_INFO_STREAM("Default cell validation = " << validation);
            // }
            // break;
        }

        if(false == isNAN) validation = true;      
    }
    else
    {
        // if(checkDefaultFoothold_Debug_){
            ROS_WARN( "Cell value not valid." );
        // }         
    }

    validation = true; // all cells value is NAN        
  }
      
  return validation;
}
// 落足点搜索矩形区域
grid_map::Polygon FootholdPlanner::getSearchPolygon(geometry_msgs::PointStamped center,
                                                    float radius){
    grid_map::Polygon polygon; 
    grid_map::Position center_vertex,leftup_vertex, rightup_vertex, rightdown_vertex, leftdown_vertex;

    
    center_vertex.x()=center.point.x+radius;
    center_vertex.y()=center.point.y + 0.5*radius;
    // leftup_vertex.x() = center.point.x + radius;
    // leftup_vertex.y() = center.point.y + 0.5*radius; 
    // rightup_vertex.x() = center.point.x + radius;
    // rightup_vertex.y() = center.point.y - 0.5*radius; 
    // rightdown_vertex.x() = center.point.x - radius;
    // rightdown_vertex.y() = center.point.y - 0.5*radius; 
    // leftdown_vertex.x() = center.point.x - radius;
    // leftdown_vertex.y() = center.point.y + 0.5*radius; 

    polygon.addVertex(center_vertex);
    // polygon.addVertex(leftup_vertex);
    // polygon.addVertex(rightup_vertex);
    // polygon.addVertex(rightdown_vertex);
    // polygon.addVertex(leftdown_vertex);  
    polygon.setFrameId(mapFrame_); 
    
    return(polygon);
}

//落足点规划检查
 bool FootholdPlanner::checkFootholdUseCentroidMethod(grid_map::GridMap& gridmap,
                                        geometry_msgs::PointStamped defaultFoothold)
    {

    grid_map::GridMap map;
    // 足端矩形搜索区域，长=搜索圆的直径，宽=搜索圆的半径
    cout<<endl<<"Get sub gridmap of the searching region ..."<<endl;
    grid_map::Length rect;

    rect.x() = searchRadius_*2;
    rect.y() = searchRadius_;

    bool isSuccess;
    grid_map::Position p;

    // STEP(1). 获取足端搜索区域对应的gridmap
    // http://docs.ros.org/kinetic/api/grid_map_core/html/classgrid__map_1_1GridMap.html#abc25d137c21d60f93b34b7b921b781a7
    p.x() = defaultFoothold.point.x-0.2;
    p.y() = defaultFoothold.point.y;
    
    cout<<"P.X"<<p.x()<<endl;
    
    map = gridmap.getSubmap(p, rect, isSuccess); 
    if(!isSuccess){
        ROS_ERROR("Can not get map.");
        return false;
    }
    // else
    // {
        // printf("sub gridmap length: %f * %f m.\n",
        // map.getLength().x(), map.getLength().y());
        grid_map::Index index;
        if(map.getIndex(p, index))
            printf("Index of sub gridmap: (%i, %i)\n", index(0), index(1));
        if(gridmap.getIndex(p, index))
            printf("Index of gridmap: (%i, %i)\n", index(0), index(1));
    // // }
    
    // STEP(2). 遍历矩形搜索区域
    //         // 2.1 先整体遍历矩形搜索区域，若所有cell均可通行，则矩形中心就是最终落脚点
    cout<<endl<<"Checking sub gridmap and get the centroid ..."<<endl;
    bool wholeRegionValid = false;
    grid_map::Matrix& data = map["traversability"];
    for(grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it){
        const int i = it.getLinearIndex();

        // cout<<
        // printf("Index of gridmap: (%i)\n", data(i));
        cout << i << "  " << data(i) << "  ";
        // if(data(i) < defaultFootholdThreshold_){
        //     wholeRegionValid = false;
        //     break;
        // }
        // wholeRegionValid = true;    
    }

    return 1;
    }

//主函数
int main(int argc, char **argv)
{   
    FootholdPlanner FootholdPlanner_zyt;
    // FootholdPlanner_zyt.readParameters();
    FootholdPlanner_zyt.FootholdPlannerInit(argc,argv);
    FootholdPlanner_zyt.SubMsg();
    // FootholdPlanner_zyt.checkFootholdUseCentroidMethod(gridmap_,)    

    // FootholdPlanner_zyt.checkFootholdUseCentroidMethod(FootholdPlanner_zyt.gridmap_,FootholdPlanner_zyt.defaultFoothold_);

    //  clock_t while_startTime, while_endTime;
    //  cout<<"the start of the main function"<<endl;
    //  ros::init(argc,argv,"map");

    //  ros::NodeHandle n;
    //  ros::Subscriber gridmapSub_;

    // gridmapSub_ = m.subscribe("/traversability_estimation/traversability_map", 20, gridmapCallback);

    //  path_pub = n.advertise<nav_msgs::Path>("trajectory_odom", 10, true);
     // ros::Subscriber odom_Sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, odom_Callback);  //订阅里程计话题信息
    //  ros::Subscriber odom_Sub = n.subscribe<nav_msgs::Odometry>("/odom_now", 10, odom_Callback);  //订阅里程计话题信息
    //  ros::Subscriber sub = n.subscribe("/elevation_mapping/elevation_map",1000,chatterCallback); 
    //  ros::Subscriber sub = n.subscribe("my_topic", 1, &Foo::callback, &foo_object);
    //  ros::Subscriber gridmapSub_ = n.subscribe("/traversability_estimation/traversability_map", 20, &FootholdPlanner::gridmapCallback,this);
    //  ros::Subscriber gridmapSub_ = n.subscribe("");
     /*发布四条腿的，四个不同的足端*/
    //  ros::Publisher basic_pubP = n.advertise<T1::FootresultMsg>("/Footresult_infoP", 10);
  

     // 设置循环的频率
     ros::Rate loop_rate(10); //这个频率要协调好
     while (ros::ok)
     {
          // while_startTime = clock();
          // cout<<"the start of spinonce of the main function"<<endl;
          ros::spinOnce();

        //   Footresult_msgP.resultFootholdx = this_pose_stamped.pose.position.x+0.2 ;
        //   Footresult_msgP.resultFootholdy = this_pose_stamped.pose.position.y ;
        //   Footresult_msgP.resultFootholdz = 0;

       

          // ros::Publisher basic_pub = n.advertise<elevation_mapping::FootresultMsg>("/Footresult_info", 10);
        //   basic_pubP.publish(Footresult_msgP);
          // basic_pubLF.publish(Footresult_msgLF);
          // basic_pubRF.publish(Footresult_msgRF);
          // basic_pubLH.publish(Footresult_msgLH);
          // basic_pubRH.publish(Footresult_msgRH);

          

          loop_rate.sleep();
          
     }

     return 0;

 }
