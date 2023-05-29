#include "foo_and_bod_adj_and_map.h"


void FooTipAndBodAdjMap::setInitEndEfforeAndPBias(Eigen::Vector3d xyz_,double * bias_xyz,double WB_X,double WB_Y)
// void foo_and_bod_adj_and_map::setInitEndEfforeAndPBias(double x,double y,double z,double bx,double by,double bz,double WB_X,double WB_Y)
{
    double xyz[3];
    for (int i = 0; i < 3; i++)
    {
        xyz[i]=xyz_(i);
    }
    
    //给每个笛卡尔坐标系定义一个足端初始点的位置，轨迹会在这个足端初始点位置叠加,依次为，长度、宽度、高度方向，即x、y、z
    end_effore<<xyz[0],xyz[1],xyz[2];

    //%足端位置偏离
    // PositonBias_<<bias_xyz[0],bias_xyz[1],bias_xyz[2],1;

    // %笛卡尔坐标原点相对COM的平移
    Body_P_<<WB_X,WB_Y,0;

    //%用于姿态调整的矩阵
    //得到在各腿笛卡坐标系中，足端位置在x,y方向上的位置信息
    OB_<<xyz[0],xyz[1],0,1;

    OB_temp_<<WB_X,WB_Y,0,1;

    // OA为各腿笛卡尔坐标系原点相对于机身中心O的位置，OO_为机身坐标系相对于世界坐标系的位置
    OA_<<WB_X,WB_Y,0,1;

    //得到在机身原点O的坐标系中，足端位置在x,y方向上的位置信息
    OB_=OB_+OB_temp_;

    // std::cout<<b<<std::endl;
}

Eigen::Vector3d FooTipAndBodAdjMap::trajAndAdjustMapping(Eigen::Vector3d traj)
{
    // RRR_yaw_ns=RRR_YAW;RRR_roll_ns=RRR_ROLL;RRR_pitch_ns=RRR_PITCH;
    // x_deviate_ns=X_DEVIATION;y_deviate_ns=Y_DEVIATION;z_deviate_ns=Z_DEVIATION;

    //调整_deviate，可以调整机身原点相对于世界坐标系的位置，从而达到机身在x,y,z方向偏移的目的
    // OA为各腿笛卡尔坐标系原点相对于机身中心O的位置，OO_为机身坐标系相对于世界坐标系的位置
    // OO_<<0+X_DEVIATION,0+Y_DEVIATION,-fabs(end_effore(2))+Z_DEVIATION,1;
    OO_<<0+X_DEVIATION,0+Y_DEVIATION,-end_effore(2)+Z_DEVIATION,1;

    //RRR_yaw，RRR_roll，RRR_pitch，传入机身相对于世界的齐次变换矩阵RRR中，可以改变机身的姿态角
    RRR<<cos(RRR_YAW)*cos(RRR_PITCH),cos(RRR_YAW)*sin(RRR_PITCH)*sin(RRR_ROLL)-sin(RRR_YAW)*cos(RRR_ROLL),cos(RRR_YAW)*sin(RRR_PITCH)*cos(RRR_ROLL)+sin(RRR_YAW)*sin(RRR_ROLL),OO_(0),
    sin(RRR_YAW)*cos(RRR_PITCH),sin(RRR_YAW)*sin(RRR_PITCH)*sin(RRR_ROLL)+cos(RRR_YAW)*cos(RRR_ROLL),sin(RRR_YAW)*sin(RRR_PITCH)*cos(RRR_ROLL)-cos(RRR_YAW)*sin(RRR_ROLL),OO_(1),
    -sin(RRR_PITCH),cos(RRR_PITCH)*sin(RRR_ROLL),cos(RRR_PITCH)*cos(RRR_ROLL),OO_(2),
    0,0,0,1;
    
    // %乘旋转矩阵可以平移
    _Rz<<cos(-rzz),-sin(-rzz),0,
            sin(-rzz),cos(-rzz),0,
            0,0,1;  

    //得到足端在笛卡尔坐标下单位置
    //RRR*OA可以得到包含姿态角信息、机身高度信息的位置矢量，加上OB位置矢量，即可得到笛卡尔坐标系下的足端位置矢量
    // AB_=PositonBias_-(RRR*OA_)+OB_;
    AB_=PositonBias_-(RRR*OA_)+OB_;

    Frame_Body=traj;

    // %乘旋转矩阵可以平移
    Frame_Body=_Rz*Frame_Body;

    //笛卡尔坐标系足端位置/轨迹,即足端轨迹=足端位置矢量+质心COM的轨迹
    // eg: inveres_kinematic_math2(leg_number,Frame_rb(0),Frame_rb(1),Frame_rb(2));
    end_effore_<<AB_(0),AB_(1),AB_(2);
    // Frame_=Frame_Body*step_amplitude+end_effore_;
    Frame_=Frame_Body+end_effore_;

    // Postion[0]=Frame_(0);
    // Postion[1]=Frame_(1);
    // Postion[2]=Frame_(2);


    // %笛卡尔坐标下足端相对于COM的位置
    Frame_EndEffore_=Frame_+Body_P_;   

    // printf(" xyz: %f %f %f ryp: %f %f %f \n",X_DEVIATION, Y_DEVIATION, Z_DEVIATION, RRR_YAW, RRR_ROLL,RRR_PITCH  );

    // return Postion;  
    return Frame_;  
}

double * FooTipAndBodAdjMap::footTipPosRefToBody(void)
{

    Postion[0]=Frame_EndEffore_(0);
    Postion[1]=Frame_EndEffore_(1);
    Postion[2]=Frame_EndEffore_(2);

    return Postion;
}
 
Eigen::Vector3d FooTipAndBodAdjMap::legOriRefToBody(void)
{
    Eigen::Vector3d ppp;
    ppp<< Body_P_(0),Body_P_(1),Body_P_(2);
    // Postion[0]=Body_P_(0);
    // Postion[1]=Body_P_(1);
    // Postion[2]=Body_P_(2);

    return ppp;
}
 
void FooTipAndBodAdjMap::msgShow()
{
    // std::cout<<RRR<<std::endl;

    // printf("traj:%f %f %f \n ",traj[0],traj[1],traj[2]);
    // printf("Frame_Body:%f %f %f \n ",Frame_Body(0),Frame_Body(1),Frame_Body(2));
    // printf("AB_:%f %f %f \n ",AB_(0),AB_(1),AB_(2));

    // printf("desPostionRefFuselageOrien:%f %f %f \n ",Frame_EndEffore_(0),Frame_EndEffore_(1),Frame_EndEffore_(2));  
    // printf("desPostionRefLegOrien:%f %f %f \n ",Frame_(0),Frame_(1),Frame_(2));   
}
