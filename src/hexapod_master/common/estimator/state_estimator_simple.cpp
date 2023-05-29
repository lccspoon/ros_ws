#include "state_estimator_simple.h"
#include <pthread.h>

// 构造函数，对常数矩阵进行初始化
STAEST_SIMPLE::STAEST_SIMPLE () 
{
    // constructor
    eye3.setIdentity();
    // C is fixed
    C.setZero();    //! 测量矩阵C 固定  ->H
    for (int i=0; i<NUM_LEG; ++i) 
    {
        C.block<3,3>(i*3,0) = -eye3;  //-pos
        C.block<3,3>(i*3,6+i*3) = eye3;  //foot pos
        C.block<3,3>(NUM_LEG*3+i*3,3) = eye3;  // vel
        C(NUM_LEG*6+i,6+i*3+2) = 1;  // height z of foot
    }

    // set A to identity
    A.setIdentity();

    // set B to zero
    B.setZero();

    assume_flat_ground = true;  //! 假设平地

}

// 初始化状态空间
void STAEST_SIMPLE::init_state(Eigen::Matrix<double,3,6> foot_act_pos_ref_body,Eigen::Matrix<double,3,3>root_rot_mat) 
{
    // filter_initialized = true;

    //! 状态先验估计 set initial value of x
    x.setZero();
    Eigen::Matrix<double,3,1> pos=foot_act_pos_ref_body.block<3, 1>(0,0);
    x.segment<3>(0) << 0, 0, -pos(2);  //! 质心位置初始化，有个高度初值　　　

}

//lcc　速度可以用，位置不可用　飘的厉害．
void STAEST_SIMPLE::update_estimation(
                            Eigen::Matrix<double,3,1> imu_ang_vel, 
                            Eigen::Matrix<double,3,1> imu_acc,
                            Eigen::Matrix<double,3,3> root_rot_mat,
                            Eigen::Matrix<double,3,6> foot_act_pos_ref_body, 
                            Eigen::Matrix<double,3,6> foot_act_vel_ref_body,
                            double dt,
                            int movement_mode,
                            Eigen::Matrix<double,6,1> foot_force_z,
                            Eigen::Matrix<double,6,1> touch_dowm_scheduler) 
{

    //lcc 20230424: 我自己简单加了一个触地判断,没用到足端力
    //         //lcc 摆动时，estimated_contacts=0;
    //         //lcc 支撑时，estimated_contacts=1;
    if(movement_mode=='w' || movement_mode=='q' || movement_mode=='e' ||   //lcc　表示机器人运动模式
        movement_mode=='a' || movement_mode=='s' || movement_mode=='d' )
    {  
        printf("move mode\n");
        for (int i = 0; i < NUM_LEG; ++i) 
        {
            if(touch_dowm_scheduler(i)==0) //lcc 20230428: 在cpg的touch_dowm_scheduler中，　=0时支撑态;  =1 是摆动态；
            {
                estimated_contacts[i] =1;     //lcc 支撑时，estimated_contacts=1; 
            }
            else
            {
                estimated_contacts[i] =0;   //lcc 摆动时，estimated_contacts=0;
            }
        }    
    }
    else if(movement_mode=='c')  //lcc　机器人蹲
    {   
        printf("squat mode\n");
        for (int i = 0; i < NUM_LEG; ++i) 
        {
            estimated_contacts[i] =0;     //lcc 摆动时，estimated_contacts=0;
        }  
    }
    else                        //lcc　机器人站立，六腿着地
    {   
        printf("stand mode\n");
        for (int i = 0; i < NUM_LEG; ++i) 
        {
            estimated_contacts[i] =1;     //lcc 支撑时，estimated_contacts=1; 
        }  
    }

    int touch_down_leg_number=0;
    for (int i=0; i<NUM_LEG; ++i) 
    {
        if(estimated_contacts[i]==1)
            touch_down_leg_number=touch_down_leg_number+1;
    }
    printf("touch_down_leg_number:%d\n",touch_down_leg_number);


    Eigen::Vector3d u = root_rot_mat * imu_acc + Eigen::Vector3d(0, 0, 0);//lcc
    std::cout<<"u:"<<u.transpose()<<std::endl;
    std::cout<<"imu_acc:"<<imu_acc.transpose()<<std::endl;
    std::cout<<"imu_ang_vel:"<<imu_ang_vel.transpose()<<std::endl;



    Eigen::Vector3d body_vel;
    body_vel.setZero();
    for (int i=0; i<NUM_LEG; ++i) 
    {
        Eigen::Vector3d fk_pos = foot_act_pos_ref_body.block<3,1>(0,i);    //lcc foot_act_pos_ref_body,机身系下足端位置

        //! 足端在全局坐标系下的速度 = 在body坐标系下的速度 + 质心在全局坐标系下的速度
        Eigen::Vector3d leg_v = -foot_act_vel_ref_body.block<3,1>(0,i) - skew(imu_ang_vel)*fk_pos;    //? 世界坐标系下的足端速度 吗？ //lcc foot_act_pos_ref_body,机身系下足端位置

        //lcc 摆动时，estimated_contacts=0,(1 - estimated_contacts[i]) * 1e3=1e3,直接拉满；　
        //lcc 支撑时，estimated_contacts=1,(1 - estimated_contacts[i]) * 1e3=0；　
        y.block<3,1>(NUM_LEG*3+i*3,0) =  estimated_contacts[i]*root_rot_mat*leg_v;      // vel estimation
        body_vel=body_vel+ ( estimated_contacts[i]*root_rot_mat*leg_v );  
    }
    x.block<3,1>(3,0)= ( body_vel/touch_down_leg_number )*0.5  + ( u*dt )*0.5;  //lcc 速度估计 ; *0.5　表示各相信一半

    x.segment<3>(0)=x.segment<3>(0)+( u*dt ); //lcc 位置估计




    //! 实际测量过程 actual measurement
    // for (int i=0; i<NUM_LEG; ++i) 
    // { 
    //     Eigen::Vector3d fk_pos = foot_act_pos_ref_body.block<3,1>(0,i);    //lcc foot_act_pos_ref_body,机身系下足端位置
    //     y.block<3,1>(i*3,0) = root_rot_mat*fk_pos;   // 世界坐标系下足端的位置fk estimation

    //     //! 足端在全局坐标系下的速度 = 在body坐标系下的速度 + 质心在全局坐标系下的速度
    //     Eigen::Vector3d leg_v = -foot_act_vel_ref_body.block<3,1>(0,i) - skew(imu_ang_vel)*fk_pos;    //? 世界坐标系下的足端速度 吗？ //lcc foot_act_pos_ref_body,机身系下足端位置
        
    //     //! 支撑相的时候就用算出来的足端速度（因为足端的速度是根据足端的位置算出来的，而此时足端的位置是可靠的），摆动相的时候（足端位置和速度不可靠）用质心的速度
    //     //lcc 摆动时，estimated_contacts=0,(1 - estimated_contacts[i]) * 1e3=1e3,直接拉满；　
    //     //lcc 支撑时，estimated_contacts=1,(1 - estimated_contacts[i]) * 1e3=0；　
    //     y.block<3,1>(NUM_LEG*3+i*3,0) = (1.0-estimated_contacts[i])*x.segment<3>(3) +  estimated_contacts[i]*root_rot_mat*leg_v;      // vel estimation

    //     //! 支撑相的时候默认足端高度是0（这个假设yxy的论文中提到过）,摆动相的时候通过质心高度和编码器得到的足端位置计算（加入足端z，是因为他会对质心的运动状态有影响）
    //     //lcc 摆动时，estimated_contacts=0,(1 - estimated_contacts[i]) * 1e3=1e3,直接拉满；　
    //     //lcc 支撑时，estimated_contacts=1,(1 - estimated_contacts[i]) * 1e3=0；　
    //     y(NUM_LEG*6+i) = (1.0-estimated_contacts[i])*(x(2)+fk_pos(2)) + estimated_contacts[i]*0;                               // height z estimation
    // }

    //lcc :把估计结果，返回到state中;
    //    std::cout << x.transpose() <<std::endl;
    state.estimated_root_pos = x.segment<3>(0);     //! 状态估计的得到质心位置
    state.estimated_root_lin_vel = x.segment<3>(3);     //! 状态估计得到的质心速度

}
