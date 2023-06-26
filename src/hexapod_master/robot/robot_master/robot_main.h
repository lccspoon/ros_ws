/**
* @file robot_main.h
* @brief 这个头文件声明了所有robot_master文件里所有东西，而这些声明的具体定义分布在各个.cpp中;是robot_master文件夹的核心.h文件。
* @details 主要声明了两个类：Hexapod  ，RobotInterface。
* @author lcc
* @date date at : 20230329
*/
#ifndef _RROBOT_MAIN_H
#define _RROBOT_MAIN_H
#include "../main_helper.h" 
#include "robot_kinematic.h"
#include "robot_parameter.h"
#include "../../user/remote_monitor/keyboard_ctrl.h"
#include "../../common/scheduler/cpg_scheduler.h"
#include "../../common/scheduler/simple_scheduler.h"
#include "../../common/foot_swing_trajectory/foo_and_bod_adj_and_map.h"
#include "../../common/foot_swing_trajectory/lagrange_interpolator.h"
#include "../../common/foot_swing_trajectory/neural_bezier_curve.h"
#include "../../common/utilities/protection.h"
#include "../../common/utilities/linear_trans.h"
#include "../../common/utilities/bubble_sort.h"
#include "../../common/utilities/timer.h"
#include "../../common/utilities/pid_controller.h"
#include "../../common/estimator/state_estimator_yxy.h"
#include "../../common/estimator/state_estimator_simple.h"
#include "../../common/estimator/contact_detection_simple.h"
#include "../../common/estimator/xzb_leg_odometer.h" //xzb230512 add
#include "../../user/pd/pd_ctrl.h"
#include "../../user/ada_control/ada_control.h"
#include "../hardware/hardware_bridge.h"
#include "../simulation/simulation_bridge.h"




/**
* @brief 跟据宏定义控制机器人的功能、模式，请细看。很重要！！！
* @author lcc
*/
#define HARD_WARE 2  // 1：开硬件，不开gazebo 2：关硬件;开gazebo  
#define SEQ_CHOKE 2  // 1:开启时序阻塞，如果接收消息的时序不更新，那么就会让线程死在 Hexapod::recData()中等待时序更新  2：不开启时序阻塞
#define DEBUG 0  // 1 在调试   ； 0 不调试
#define ADAPTIV_FLAG 1  // 自适应控制算法　0 关   ； 1 开
#define ONLY_Quadruped 0  // 只能用四足步态　0 关   ； 1 开

#define SIM_CTRL_MODE 1   // 仿真算法：1 -> position control  2-> pd control
#define SIM_PROTECT 2 // 1:开启保护　　２：不保护
#define TXT_FLAGE 1   // 1 -> load  0-> no load
#define STRUCT 3 // 选择仿真模型 1: yobotics   ； 2: new  3:newtwo

#if HARD_WARE==1
    #include "../hardware/imu/imu_main.h"//xzb230522
#elif HARD_WARE==2
    extern ROSLegTopicHandle GazeboSim;
#endif
extern KeyBoardControl KeyBoardCtrl;

extern A1BasicEKF StateEstimator;  //lcc 20230428
extern bool _state_estimator_flag;  //lcc 20230428
extern std::mutex _state_estimator_mut;

/**
* @brief 机器人的硬件接口
* @note 好像没多大用处
* @author lcc
*/
class RobotInterface
{
    private:

    public:
        int data_sequence=0;
        double * rec_data_pos=new double[18],* rec_data_vel=new double[18],* rec_data_tor=new double[18];
};

extern RobotInterface HexInt;

/**
* @brief 封装了机器人类Hexapod的所有数据和函数接口。
* @details 提供的函数接口的具体定义分布在：robot_controller（用于定义机器人用的控制器），robot_motion（用于规定机器人的动作）
* @details        robot_runner（机器人的主体运行框架），  robot_utilities（定义其他一些有用的函数）            
* @author lcc
*/
class Hexapod:public RobotParam
{
    private:
        simple_scheduler _SimpleScheduler[6];
        LagrangeInterpolator _LagrangeInterpolator[6];

        kinematic _kinematic[6];

        neural_bezier_curve neur_bezier[6];
        CPG _Cpg;
        FooTipAndBodAdjMap _FooBodAdjMap[6];
        TimeMteter _Tim1;

        contact_detection_simple ContactSimple;

        linear_trans _start_pos_conver[6];

        linear_trans _set_static_pos_conver[6];

        linear_trans _step_length_k_conver[6];
        linear_trans _step_hight_k_conver[6];

        linear_trans trajctopry_temp;

        DataUnusualProtect _dataUnuProtect[6];

        pd_ctrl _LfPdCtrl,_LmPdCtrl,_LbPdCtrl,_RfPdCtrl,_RmPdCtrl,_RbPdCtrl;

        ada_ctrl _LfAdaCtrl,_LmAdadCtrl,_LbAdaCtrl,_RfAdaCtrl,_RmAdaCtrl,_RbAdaCtrl;//zyt_control 2023.4.19

        xzb_leg_odom _XzbEstimator; //xzb230512 add

        pid_controller attitude_pid_ctrl[3];
        linear_trans attitude_conver[3];
        linear_trans deviation_conver[3];

        BubbleSort foot_cross_hight_sort, foot_ditch_deepth_sort;
        BubbleSort get_original_step_higtt_sort[6], get_original_step_length_sort[6];
        BubbleSort get_des_step_hight_sort[6], get_des_step_length_sort[6];

        neural_bezier_curve neur_bezier_lift_curve[6];

        Eigen::Matrix<double,1,6> leg_real_cpg_signal;
    public:

        /**
        * @brief 以下接口声明了机器人的运行结构的函数
        * @author lcc
        */
        bool start_thread_flag=true,get_joint_pos_flag=false;
        int  joint_pos_run_count=0,motor_pos_set_count=0,motor_pos_ach_count_flag=0;
        int  t_test_key=0;
        void run(int argc, char *argv[]);
        void recData();
        void recDataHandling(void);
        void msgShow(void);
        void simMsgPub(void);
        void realRobMsgPub(void);

        /**
        * @brief 以下接口声明了机器人的参数设置函数
        * @author lcc
        */
        void parSeting(void);
        void parInit(void);
        void setStepSize();
        void getDesPosAndVel();

        /**
        * @brief 以下接口声明了机器人的控制函数
        * @author lcc
        */
        void trajectoryPlaning(void);
        void adaController(void);   //zyt_adacontrol
        void pdController(void);
        void positionController(void);
        void realHexMsgLoadAndSend(void);
        void keyBoardControl(int key_value);

        /**
        * @brief 以下接口声明了机器人的动作函数、相关变量
        * @author lcc
        */
        bool stand_done_flag=false;
        void setSixLegSteAmplitude(double lf_step_amplitude,double lm_step_amplitude,double lb_step_amplitude,
                                double rf_step_amplitude,double rm_step_amplitude,double rb_step_amplitude,double switch_period);
        void setSixLegRzz(double lf_rzz,double lm_rzz,double lb_rzz,
                          double rf_rzz,double rm_rzz,double rb_rzz,double switch_period);
        void setStandPose(void);
        void robTripleWalk(void);
        void robQuadruWalk(void);
        void robHexapodWalk(void);
        void robStand(double switch_period);
        void robSquat(double switch_period);
        void robGoAhead(double amplitude);
        void robGoBack(double amplitude);
        void robGoLeft(double amplitude);
        void robGoRight(double amplitude);
        void robTurnRightAround(double amplitude);
        void robTurnLeftAround(double amplitude);
        void attitudeAdjust(void);
        void kpkdSwitch(void);

        void adaptive_control(void);
        void liftReaction(void);
        void liftFollowReaction(void);
        void liftFollowTraject(void);
        void changeBezierShape(int i);
        void dowmwardReaction(void);
        void dowmwardFollowReaction(void);
        void dowmwardFollowTraject(void);
};

#endif