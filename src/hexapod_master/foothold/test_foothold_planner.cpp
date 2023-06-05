#include"test_foothold_planner.h"
// #include"ros/ros.h"

using namespace FootHoldPlanner;
double mapResolution = 0.02; // gridmap分辨率

geometry_msgs::PointStamped P1;

/*----------------------回调函数-------------------------*/

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

void FootholdPlanner::chatterCallback(const grid_map_msgs::GridMap& message)
{
    clock_t chatter_startTime, chatter_endTime;
     chatter_startTime = clock();
     //ROS_INFO("%f",message->data.std_msgs.Float32MultiArray.float32());

     // Convert message to map.
     GridMap gridMap;
     geometry_msgs::Point32 point;
     //三种不同类型的点云
     std::string pointLayer = ("elevation", "upper_bound", "lower_bound") ;
     sensor_msgs::PointCloud2 pointCloud;//点云数据结构PointCloud2
     sensor_msgs::PointCloud out_pointcloud;//点云数据结构PointCloud
     pcl::PointCloud<pcl::PointXYZ> cloud;
     
     //68-71是为了获取grid_map的x,y引入的
     //https://docs.ros.org/en/kinetic/api/grid_map_ros/html/classgrid__map_1_1GridMapRosConverter.html
     GridMapRosConverter::fromMessage(message, gridMap);//将 ROS 网格地图消息转换为网格地图对象。
        //  
         
    grid_map::GridMap map;
    grid_map::Length rect;//长宽
    double searchRadius_=0.1;

    //     rect.x() = searchRadius_*2;
    //     rect.y() = searchRadius_;
    rect.x() = 0.2;
    rect.y() = 0.2;

    bool isSuccess;
    grid_map::Position PLF, PRF, PLH, PRH, PLM, PRM; // submap中心位置————这个p的位置，需要随着机器人变动
    grid_map::Position P;/*里程计位置*/

    P.x() =  this_pose_stamped.pose.position.x+0.12;
    P.y() =  this_pose_stamped.pose.position.y;

    default_foot.x() = P.x() + 0.03;
    default_foot.y() = P.y();

    foot_selection_part(gridMap, P, default_foot, rect);
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
    chatter_Sub = m.subscribe("/elevation_mapping/elevation_map",100,&FootholdPlanner::chatterCallback,this); 
    // gridmapSub_ = m.subscribe("/traversability_estimation/traversability_map", 100, &FootholdPlanner::gridmapCallback,this);
    // odom_Sub = m.subscribe<nav_msgs::Odometry>("/odom_now", 10, &FootholdPlanner::odom_Callback,this);
 

//
}
//落足点选择
void  FootholdPlanner::foot_selection_part(GridMap gridMap, grid_map::Position p, grid_map::Position default_foot, grid_map::Length rect)
{
    geometry_msgs::Point32 resultFoothold;
     grid_map::GridMap map;
     bool isSuccess;
     map = gridMap.getSubmap(p, rect, isSuccess);

     cout << "map的实际大小" << map.getSize() << endl;

     // GridMapRosConverter::toPointCloud(map,
     //                                   pointLayer,
     //                                   pointCloud);
     // sensor_msgs::convertPointCloud2ToPointCloud(pointCloud, out_pointcloud);

     grid_map::Size num;
     num = map.getSize(); //获得map的行列大小
     int bottomRow = num(0) - 1;
     int rightCol = num(1) - 1;
     cout << "bottomRow" << bottomRow << endl;
     cout << "rightCol" << rightCol << endl;
     //计算中间的索引位置
     double pz;
     pz = map.atPosition("elevation", p);
     cout << "pz  " << pz << endl;
     grid_map::Matrix &data = map["elevation"];

     cout << endl<< "高程值    " << endl;
     for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
     {
          const Index index(*it);
          //      cout<< "The value at index "<<index.transpose();
          const int i = it.getLinearIndex(); //这里相当于给出it对应的索引
          // if(!isnan(data(i)))
          // {
          //      data(i) = 1000;
          // }
          cout << i << "  " << data(i) << "  ";
          //   cout<<*it<<"  ";
     }

     grid_map::Position resultxy; //落足点评估的结果
                                  // geometry_msgs::Point32 resultFoothold;//目标落足点
     int index = -1;              //落足点位置的索引
     double min_val_point = 1000; //

     int sum_size = num(0) * num(1);                              //总的数据量
     double max_range_motion = 10, max_bump, max_step, max_slope; //假设的最大运动范围
     float diff = 1, crticle_val = 50;
     double bump_res[sum_size], slope_res[sum_size], step_res[sum_size], distance_res[sum_size], point_res[sum_size], front_res[sum_size];

     //评估值初始化
     for (int i = 0; i < sum_size; i++)
     {
          point_res[i] = 1000;
          bump_res[i] = 1000;
          slope_res[i] = 1000;
          step_res[i] = 1000;
          distance_res[i] = 1000;
          front_res[i] = 1000;
          if(isnan(data(i))) data(i) = 10000;
     }

     // 1.计算高度差(计算当前落足点在子图内的高度差)
     for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
     {
          double temp_step = 0;
          const int i = it.getLinearIndex();
          step_res[i] = abs(data(i) - pz);
          if(step_res[i] >0.28)
          {
               step_res[i] = 1000;
          }
     }

     cout << endl<< "输出高度差 step_res" << endl;
   
     for (int i = 0; i < sum_size; i++)
     {
          cout << step_res[i] << "  ";
     }

     // 2.坐标点之间的距离(这里距离是否对还需要验证下)//排除后端的数据点
     for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
     {
          double temp_step = 0;
          const int i = it.getLinearIndex();

          // //这里自己先验证下it.x  it.y的值
          // cout<<"*it"<<*it<<endl;//这里显示的是map子图的行数

          grid_map::Position pxy;
          // const Index index(*it);
          map.getPosition(*it, pxy); //由索引（单元格位置的 x，y）指定的单元格的 2d 位置。

          //排除后端的数据点
          if (pxy.x() <= p.x())  /*p为中心位置， pxy为单元格的位置*/
          {
               front_res[i] = 0;
          }
          else
          {
               front_res[i] = 0;
          }

          distance_res[i] = sqrt(pow((data(i) - pz), 2) + pow(pxy.x() - p.x(), 2) + pow(pxy.y() - p.y(), 2));
          //  cout<<*it <<"  ";
     }
     cout << endl << "输出结果 distance_res" << endl;
     for (int t = 0; t <sum_size ; t++)
     {
          cout << distance_res[t] << "  ";
     }

     // 3.遍历矩形区域获得凹凸度数据, 斜度数据
     cout << "check the result of step" << endl;
     for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
     {
          //边缘数据不处理（直接设定为边界值）
          const int i = it.getLinearIndex(); //这里相当于给出it对应的索引
          //对边缘数据的处理(这里还要加一个高度的约束，空间点距离的约束)
          // cout<<"对边缘数据的处理"<<endl;//这里的设定要检查下，是否排除了所有不可以计算的值
          if (i <= rightCol || (i % num(1)) == 0 || (i % num(1)) == rightCol || (sum_size - i) <= num(1) || (distance_res[i] >= max_range_motion))
          {
               //
               // step_res[i] = crticle_val+1;
               slope_res[i] = crticle_val + 1;
               bump_res[i] = crticle_val + 1;
               continue;
          }

          //   cout<<"非边缘数据"<<endl;x
          //非边缘数据进行处理
          double temp_bump = 0, temp_slope = 0;
          // K1[i,j]=Z(i,j)-Z(i+k,j+l)
          //两层循环，一层列数，一层行数
          int mincol = -num(1), maxcol = num(1);
          // cout<<"两层循环，一层列数，一层行数"<<endl;
          for (int j = mincol; j <= maxcol; j = (j + num(1))) //行
          {
               for (int k = -1; k <= 1; k++) //列
               {
                    int l = i + j + k; //这里出错，是由于l的设定有问题，l会出现负数
                                       //   double res = data(i);
                                       //  cout<<"内部数据"<< l<<endl;
                                       //   int l = i+j+k;
                    
                    temp_bump = temp_bump + (data(l) - data(i));
                    temp_slope = temp_slope + abs(data(l) - data(i));
               }
          }
          //               cout<<"外部数据"<< i<<endl;
          // cout<<"输出结果"<<data(2)<<"get"<<endl;
          slope_res[i] = temp_slope;
          bump_res[i] = temp_bump;
     }

     cout << endl<< "输出结果slope_res" << endl;
     for (int t = 0; t <sum_size ; t++)
     {
          cout << slope_res[t] << "  ";
     }

     cout << endl<< "输出结果bump_res" << endl;
     for (int t = 0; t <sum_size ; t++)
     {
          cout << bump_res[t] << "  ";
     }

     // 4. 综合评估上面的几个因素，选出最优的落足点(取w的调节因子为1/3)(上面还要设置一个地形各个因素的极限值)

     cout << "获取目标点的坐标值" << endl;
     cout << "temp_point" << endl;
     // 4.1 选出了最小值的点(ok)
     for (int t = 0; t < sum_size; t++)
     {
          double temp_point = (slope_res[t] + bump_res[t] + step_res[t] + front_res[t] + distance_res[t]);
          if (temp_point <= min_val_point)
          {
               min_val_point = temp_point;
               index = t; //这里获取了最小值的对应索引//还要从当前的索引回到地图的x,y,z坐标值
          }
          point_res[t] = temp_point;
          // // // cout << "temp_point" << temp_point << "min_val_point" << min_val_point << endl;
     }

     //先获取默认落足点的数据，再加入判断
     grid_map::Index index_point_default;
     grid_map::Position default_foot_test;
     map.getIndex(default_foot, index_point_default); //默认落足点位置对应的索引
     int result_index = index_point_default[0] * (rightCol + 1) + index_point_default[1];
     cout << "index_point_default[0]" << index_point_default[0] << "index_point_default[1]" << index_point_default[1] << endl;
     cout << "point_res[result_index] " << point_res[result_index] << endl;

     // map.getPosition(index_point_default, default_foot_test);//默认落足点位置对应的索引

     //  cout<<"default_foot的索引"<<default_foot<<endl;
     //   cout<<"index_point_default的索引x"<<index_point_default<<"index_point_default的索引x"<<index_point_default[1]<<endl;
     // cout<<"default_foot_test的索引"<<default_foot_test<<endl; //这里最后验证的default_foot的索引和default_foot_test的索引 不一样

     // //后面自己要比较下，高程图得到的数据。和自己给的地形高度的准确性

     // //默认落足点的索引：行×列数+列数

     // int result_index = index_point_default[0]*rightCol+index_point_default[1];
     // cout<<"index_point_default[0]"<<index_point_default[0]<<"index_point_default[1]"<<index_point_default[1]<<endl;
     // cout<<"result_index索引"<<result_index<<endl;

     //  const int default_it =  index_point_default.getLinearIndex();

     //这里需判断下位置转化的数据是否争取，验证索引，坐标值

     if (point_res[result_index] <= 0.37)
     {
          map.getPosition(index_point_default, resultxy); //获取所选落足点的索引对应x,y位置//还要定义一个空间的目标落足点位置
          resultFoothold.x = default_foot.x();
          resultFoothold.y = default_foot.y();
          resultFoothold.z = data(result_index);
          cout << "resultFoothold.x" << resultFoothold.x << endl;
          cout << "resultFoothold.y" << resultFoothold.y << endl;
          cout << "resultFoothold.z" << resultFoothold.z << endl;

          ROS_INFO("Publish Person Info: name:%lf  age:%lf  sex:%lf", resultFoothold.x, resultFoothold.y, resultFoothold.z);
          // basic_shapes(geometry_msgs::Point32 resultFoothold);

        //   return resultFoothold;
     }
     else
     {
          index = index + num(1);
          cout << "最小值索引" << index << endl;
          //这里需要把所有综合的结果打印出来显示
          cout << "输出综合考虑的落足点" << endl;
          for (int t = 0; t < sum_size; t++)
          {
               cout << point_res[t] << "  ";
          }

          // 4.2 最小值的索引转换到x,y,z坐标值
          grid_map::Index index_point;
          for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
          {
               const int i = it.getLinearIndex();

               double temp_point = (slope_res[i] + bump_res[i] + step_res[i]);
               if (temp_point <= min_val_point)
               {
                    min_val_point = temp_point;
                    index_point = i; //这里获取了最小值的对应索引
               }
          }

          grid_map::Index index_point2;
          //还要从当前的索引回到地图的x,y,z坐标值（最后目标：获取局部地图的x,y()  ,z坐标值）——1.先获取对应的索引的位置（对应行数，列数） 2.
          index_point.x() = index / rightCol; //获取行数
          index_point.y() = index % rightCol; //获取列数
          cout << "px" << index_point.x() << "py" << index_point.y() << endl;
          map.getPosition(index_point, resultxy); //获取所选落足点的索引对应x,y位置//还要定义一个空间的目标落足点位置
          gridMap.getIndex(resultxy, index_point2);

          resultFoothold.x = resultxy.x();
          resultFoothold.y = resultxy.y();
          resultFoothold.z = data(index); // z的验证没有问题//还需要验证x,y的返还是否有问题

          cout << "resultFoothold.x" << resultFoothold.x << endl;
          cout << "resultFoothold.y" << resultFoothold.y << endl;
          cout << "resultFoothold.z" << resultFoothold.z << endl;

          ROS_INFO("Publish Person Info: name:%lf  age:%lf  sex:%lf", resultFoothold.x, resultFoothold.y, resultFoothold.z);
          // basic_shapes(geometry_msgs::Point32 resultFoothold);

          //验证x,y的返还是否有问题(这里我记得，直接用程序就可以完成)
          // // // cout << "验证x,y的返还是否有问题" << endl;
          // // // for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
          // // // {
          // // //      const Index index(*iterator);
          // // //      const int i = iterator.getLinearIndex();
          // // //      cout << "The value at index " << index.transpose() << " is " << data(index(0), index(1)) << "data(i)" << data(i) << endl;
          // // // }
          
          
     }

    //  return resultFoothold;
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
