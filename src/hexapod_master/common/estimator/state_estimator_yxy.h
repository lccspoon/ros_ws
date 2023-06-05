/**
* @file state_estimator_yxy.h
* @brief 一个简单的状态估计。该算法主要参考了于宪元大佬的毕业论文．
* @author lcc
* @date date at : 20230422
*/

#ifndef STATE_ESTIMATOPR
#define STATE_ESTIMATOPR

#include<iostream>
#include<eigen3/Eigen/Dense>
#include<math.h>
#include "../../common/utilities/timer.h"
#include <stdio.h>
#include <string.h>

//维度和协方差大小的定义：
#define STATE_SIZE 24   // 状态变量维度
#define MEAS_SIZE 42    // 观测变量维度
#define PROCESS_NOISE_PIMU 0.01     // 位置预测协方差
#define PROCESS_NOISE_VIMU 0.01     // 速度预测协方差
#define PROCESS_NOISE_PFOOT 0.01    // foot位置预测协方差
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001    // 足端位置测量协方差
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1      // 足端速度测量协方差
#define SENSOR_NOISE_ZFOOT 0.001    // 足端高度测量协方差


class A1BasicEKF
{
    private:
        //预测过程需要用到的变量定义：
        // state
        // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR 
        Eigen::Matrix<double, STATE_SIZE, 1> xbar; // 先验估计 estimation state
        Eigen::Matrix<double, STATE_SIZE, 1> x; // 后验估计 estimation state after process update
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // 先验估计协方差 estimation state covariance
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // 后验估计协方差 estimation state covariance after process update
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // 状态转移矩阵 estimation state transition
        Eigen::Matrix<double, STATE_SIZE, 3> B; // 输入矩阵
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // 预测协防差 estimation state transition noise

        //测量过程需要用到的变量定义:
        // observation
        // 0 1 2   FL pos residual  世界坐标系下，质心位置和足端位置之差
        // 3 4 5   FR pos residual
        // 6 7 8   RL pos residual
        // 9 10 11 RR pos residual
        // 12 13 14 vel residual from FL    不应该叫vel residual吧，就是全局坐标系下的速度
        // 15 16 17 vel residual from FR
        // 18 19 20 vel residual from RL
        // 21 22 23 vel residual from RR
        // 24 25 26 27 foot height  足端的高度 
        Eigen::Matrix<double, MEAS_SIZE, 1> y; // 实际测量 observation
        Eigen::Matrix<double, MEAS_SIZE, 1> yhat; // 测量矩阵×状态先验估计 estimated observation
        Eigen::Matrix<double, MEAS_SIZE, 1> error_y; // y-yhat estimated observation
        Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; //? S^-1*error_y
        Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C; // 测量矩阵 estimation state observation
        Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; //? S^-1*C
        Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; // 测量协方差矩阵 estimation state observation noise

        //其他相关变量
        // helper matrices 
        #define NUM_LEG 6
        Eigen::Matrix<double, 3, 3> eye3; // 3x3 identity
        Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; //这边的S矩阵是等会用来计算卡尔曼增益的一个trick，并不影响理解 Innovation (or pre-fit residual) covariance 
        Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K; // 卡尔曼增益 kalman gain

        bool assume_flat_ground = false;    //! 假设地面是平的

        // variables to process foot force
        double smooth_foot_force[NUM_LEG];
        double estimated_contacts[NUM_LEG];
        
        TimeMteter _Tim1;

    public:

        bool state_estimator_start_flag=false;

        Eigen::Matrix3d skew(Eigen::Vector3d vec) 
        {
            Eigen::Matrix3d rst; rst.setZero();
            rst <<            0, -vec(2),  vec(1),
                    vec(2),             0, -vec(0),
                    -vec(1),  vec(0),             0;
            return rst;
        }

        A1BasicEKF();
        void init_state(Eigen::Matrix<double,3,6> foot_act_pos_ref_body,Eigen::Matrix<double,3,3>root_rot_mat);
        void update_estimation(
                            Eigen::Matrix<double,3,1> imu_ang_vel, 
                            Eigen::Matrix<double,3,1> imu_acc,
                            Eigen::Matrix<double,3,3> root_rot_mat,
                            Eigen::Matrix<double,3,6> foot_act_pos_ref_body, 
                            Eigen::Matrix<double,3,6> foot_act_vel_ref_body,
                            double dt,
                            int movement_mode,
                            Eigen::Matrix<double,6,1> touch_dowm_scheduler);
        struct state_
        {
            double estimated_contacts[6];

            //世界系下：
            Eigen::Matrix<double,3,1> estimated_root_pos; //! 状态估计的得到质心位置
            Eigen::Matrix<double,3,1> estimated_root_lin_vel; //! 状态估计得到的质心速度
            Eigen::Matrix<double,3,6> estimated_foot_pos_ref_world; //! 状态估计得到的足端位置(世界系下)
            // Eigen::Matrix<double,3,1> root_pos;
            // Eigen::Matrix<double,3,1> root_lin_vel;
        }state;
 
};




#endif