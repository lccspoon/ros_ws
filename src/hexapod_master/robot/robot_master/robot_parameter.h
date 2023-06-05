#ifndef _ROBOTPARAMETER_H
#define _ROBOTPARAMETER_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include "../../common/scheduler/cpg_scheduler.h"
#include "../../common/foot_swing_trajectory/neural_bezier_curve.h"
#include "../../common/foot_swing_trajectory/foo_and_bod_adj_and_map.h"
#include "../../user/remote_monitor/keyboard_ctrl.h"
using namespace std;

class LegParameter 
{
    public:
        FooTipAndBodAdjMap _FooBodAdjMap;
};

class RobotParam  //robot_param
{
    public:

        double fuselage_length=30*0.01,fuselage_width=15*0.01;
        double LEG_DH_PARAM1=7.3*0.01, LEG_DH_PARAM2=21.1*0.01, LEG_DH_PARAM3=23*0.01;


        double _RAD1=3.1415926/180;
        double _RAD2=180/3.1415926;
        double _lf_start_pos_coe=0,_lm_start_pos_coe=0,_lb_start_pos_coe=0;  //lcc 20230514:用于机器人启动的系数
        double _rf_start_pos_coe=0,_rm_start_pos_coe=0,_rb_start_pos_coe=0;
        int rec_data_seq_last=0,rec_data_seq=0;

        double dt_s;
        double dt_ms;

        double set_x_deviation=0, set_y_deviation=0, set_z_deviation=0; 
        double set_yaw=0, set_roll=0, set_pitch=0;
        Eigen::Matrix<double,1,6> set_step_hight_k;
        Eigen::Matrix<double,1,6> set_step_length_k;
        double set_cpg_ctrl_cycle=0;
        double set_para_init_flag=1;

        int movement_mode;  //lcc 用不同的整数来表示机器人的运动模式
        int lift_adaptiv_init_flag=0;  //lcc 用不同的整数来表示机器人的运动模式
        int dowm_adaptiv_init_flag=0;  //lcc 用不同的整数来表示机器人的运动模式

        Eigen::Matrix<double,2,6> cpg_scheduler;  //lcc 20230513: 第一行为x  ;第二行为y
        Eigen::Matrix<double,1,6> cpg_touch_down_scheduler;  //lcc 20230513: 其实就是cpg的y值

        struct leg_root
        {
            Eigen::Matrix<double,1,18> joint_rec_pos;
            Eigen::Matrix<double,1,18> joint_rec_vel;
            Eigen::Matrix<double,1,18> joint_rec_tor;
            Eigen::Matrix<double,3,6> joint_des_pos;
            Eigen::Matrix<double,3,6> joint_msg_pub;  //lcc 20230513仿真中要发布的话题的消息

            Eigen::Matrix<double,3,6> foot_ret_force;  //lcc 20230514:传感器返回的足端力
            Eigen::Matrix<double,3,6> foot_act_force;  //lcc 20230514:雅克比计算的足端力

            Eigen::Matrix<double,3,6> foot_act_pos;
            Eigen::Matrix<double,3,6> foot_des_pos;

            Eigen::Matrix<double,3,6> foot_set_static_pos;
            Eigen::Matrix<double,3,6> foot_static_pos;

            Eigen::Matrix<double,3,6> foot_swing_traj;
            Eigen::Matrix<double,3,6> foot_suport_traj;

            Eigen::Matrix<double,3,6> foot_cross_traj;         //　越障轨迹
            Eigen::Matrix<double,3,6> foot_dowmward_traj;      //　下探轨迹
            Eigen::Matrix<double,1,6> foot_cross_object_est;    //　记录腿的抬升高度并且用这个高度来估计腿遇到的障碍物．
            Eigen::Matrix<double,1,6> foot_ditch_deepth_est;    //　
            Eigen::Matrix<double,3,6> foot_lift_traj;    //　
            Eigen::Matrix<double,3,6> foot_dowm_traj;    //　

            Eigen::Matrix<double,3,6> foot_trajectory;
            Eigen::Matrix<double,3,6> foot_traj_mapping_to_body;

            Eigen::Matrix<double,1,6> step_set_length; 
            Eigen::Matrix<double,1,6> step_set_hight; 
            // Eigen::Matrix<double,1,6> step_des_length; 
            // Eigen::Matrix<double,1,6> step_des_hight; 
            Eigen::Matrix<double,1,6> step_original_length; 
            Eigen::Matrix<double,1,6> step_original_hight; 
        }leg_root;

        struct body_root
        {
            Eigen::Matrix<double, 3, 1> des_vel;
            Eigen::Matrix<double, 3, 1> des_pos;

            Eigen::Matrix<double,3,1> imu_ang_vel;
            Eigen::Matrix<double,3,1> imu_lin_acc;
            Eigen::Matrix<double,4,1> imu_qua_ori;
            Eigen::Vector3d imu_eulerAngle;

            Eigen::Matrix<double,3,6> foot_act_pos;
            Eigen::Matrix<double,3,6> foot_des_pos;
            Eigen::Matrix<double,3,6> foot_act_vel; 
            Eigen::Matrix<double,3,3> root_rot_mat;   //世界系机器人旋转矩阵
            Eigen::Vector3d eulerAngle;

            Eigen::Matrix<double,3,6> leg_origin;
        }body_root;

        struct world_root
        {
            Eigen::Matrix<double,3,1> bod_des_pos_ref_world, bod_des_vel_ref_world;
            Eigen::Matrix<double,3,1> bod_act_pos_ref_world, bod_act_vel_ref_world;
            Eigen::Matrix<double,3,3> root_rot_mat;   //世界系机器人旋转矩阵
        }world_root;


        struct state_
        {
            //世界系下：
            Eigen::Matrix<double,3,1> root_pos; //! 状态估计的得到质心位置
            Eigen::Matrix<double,3,1> root_lin_vel; //! 状态估计得到的质心速度
            Eigen::Matrix<double,3,6> foot_pos_ref_world; //! 状态估计得到的足端位置(世界系下)
            Eigen::Matrix<double,1,6> contact_est_scheduler; //! 状态估计得到的足端位置(世界系下)
        }state;

};



#endif