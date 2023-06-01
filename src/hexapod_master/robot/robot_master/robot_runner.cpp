/*!
* @file robot_runner.cpp
* @brief 机器人的运行框架
* @attention 所有的声明都在robot_main.h中
* @author lcc
* @date date at : 20230329
*/
#include "robot_runner.h"
#include <iostream>
#include <fstream> // c++文件操作
#include <iomanip> // 设置输出格式
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

//lcc 20230329:一些类中静态全局变量的赋初始值
double kinematic::LEG_DH_PARAM1=0;double kinematic::LEG_DH_PARAM2=0;double kinematic::LEG_DH_PARAM3=0;

//xzb230512  全局变量用于存储需要发布的话题信息
Eigen::Vector3d vel_legodom,vel_ekf,rpy_ekf,pos_ekf;

void MySigintHandlera(int sig)//lcc 20230329: 所使用快捷键ctrl+c，中断程序运行。
{
        #if HARD_WARE==1
                // HarBri.robDataLoadExitCloesLoop();
        #endif
        printf("lcc die!\n");
        exit(0);
}

//lcc 20230329:键盘控制的线程
void *remote_monitor(void* arg)
{	
        printf("remote_monitor thread is :%lu,pid:%d\n",pthread_self(),getpid());
        while(1)
        {	
                KeyBoardCtrl.scanKeyValue();
                signal(SIGINT, MySigintHandlera );//lcc 20230329: 所使用快捷键ctrl+c，中断程序运行。
        }
}

#if HARD_WARE==2
        //lcc 20230329:ros接收消息的线程
        void *TopicLisener(void* arg)
        {	
                #if SEQ_CHOKE==1
                        printf("TopicLisener thread is :%lu,pid:%d\n",pthread_self(),getpid());
                        while ((1))
                        {       
                                ros::spin();
                        }
                #endif
        }
#endif

void *state_estimator_thread(void* arg)
{	
        printf("state_estimator_thread thread is :%lu,pid:%d\n",pthread_self(),getpid());
        while(1)
        {	

        }
}

/**
* @brief 机器人开始运行，running
* @param argc 命令参数个数 目前不起作用
* @param argv 命令参数指针数组 目前不起作用
* @author lcc
*/
void Hexapod::run(int argc, char *argv[])
{
        pthread_t RemoteControlThr;
        pthread_create(&RemoteControlThr,NULL,remote_monitor,NULL);
        kinematic::setDhParam(LEG_DH_PARAM1,LEG_DH_PARAM2,LEG_DH_PARAM3);  

        // pthread_t STAEST_THR;
        // pthread_create(&STAEST_THR,NULL,state_estimator_thread,NULL);

        #if TXT_FLAGE
                ofstream robot_force,robot_velocity,robot_postion;
                robot_force.open("./robot_force.txt", ios::out | ios::trunc);
                robot_velocity.open("./robot_velocity.txt", ios::out | ios::trunc);
                robot_postion.open("./robot_postion.txt", ios::out | ios::trunc);
        #endif

        #if HARD_WARE==1
                int output_sequnce_last=0;
                //lcc 20230506:角度软补偿
                HarBri.lf_off_set<< 3.573610*HarBri._RAD1, 0.185784*HarBri._RAD1, 2.409888*HarBri._RAD1;
                HarBri.lm_off_set<< 5.606305*HarBri._RAD1, 1.672056*HarBri._RAD1, -0.161596*HarBri._RAD1;
                HarBri.lb_off_set<< 2.196622*HarBri._RAD1, 1.366059*HarBri._RAD1, 2.185059*HarBri._RAD1;
                HarBri.rf_off_set<< -0.775921*HarBri._RAD1, -1.584628*HarBri._RAD1, -0.442633*HarBri._RAD1;
                HarBri.rm_off_set<< -0.469924*HarBri._RAD1, -3.114614*HarBri._RAD1, 2.128852*HarBri._RAD1;
                HarBri.rb_off_set<< 2.874188*HarBri._RAD1, -2.874188*HarBri._RAD1, 1.341949*HarBri._RAD1;

                int break_flag=1;
                int switch_flag=1;
                HarBri.robDataLoadExitCloesLoop();
                std::mutex mtx10;
                while(break_flag)
                {       
                        switch (KeyBoardCtrl.retKeyValue())
                        {
                                case '=':   //lcc 20230506:当前角度＋１
                                        {
                                                Eigen::Vector3d one;
                                                one.setOnes();
      
                                                HarBri.lf_des_pos=leg_root.joint_rec_pos.block<1,3>(0,0).transpose()+one*1*HarBri._RAD1; 
                                                HarBri.lm_des_pos=leg_root.joint_rec_pos.block<1,3>(0,3).transpose()+one*1*HarBri._RAD1; 
                                                HarBri.lb_des_pos=leg_root.joint_rec_pos.block<1,3>(0,6).transpose()+one*1*HarBri._RAD1; 
                                                HarBri.rf_des_pos=leg_root.joint_rec_pos.block<1,3>(0,9).transpose()+one*1*HarBri._RAD1; 
                                                HarBri.rm_des_pos=leg_root.joint_rec_pos.block<1,3>(0,12).transpose()+one*1*HarBri._RAD1; 
                                                HarBri.rb_des_pos=leg_root.joint_rec_pos.block<1,3>(0,15).transpose()+one*1*HarBri._RAD1; 

                                                HarBri.lf_des_tor.setZero();HarBri.lm_des_tor.setZero(); HarBri.lb_des_tor.setZero();
                                                HarBri.rf_des_tor.setZero();HarBri.rm_des_tor.setZero();HarBri.rb_des_tor.setZero();

                                                HarBri.lf_des_vel.setZero();HarBri.lm_des_vel.setZero();HarBri.lb_des_vel.setZero();
                                                HarBri.rf_des_vel.setZero();HarBri.rm_des_vel.setZero();HarBri.rb_des_vel.setZero();
                                                
                                                Eigen::Vector3d kp_temp, kd_temp;
                                                kp_temp<< 80, 80, 80;
                                                kd_temp<< 3, 3, 3;
                                                // kp_temp<< 20, 20, 20;
                                                // kd_temp<< 2, 2, 2;
                                                // kp_temp<< 10, 10, 10;
                                                // kd_temp<< 1, 1, 1;
                                                HarBri.lf_des_kp=kp_temp;HarBri.lm_des_kp=kp_temp;HarBri.lb_des_kp=kp_temp;
                                                HarBri.rf_des_kp=kp_temp;HarBri.rm_des_kp=kp_temp;HarBri.rb_des_kp=kp_temp;

                                                HarBri.lf_des_kd=kd_temp;HarBri.lm_des_kd=kd_temp;HarBri.lb_des_kd=kd_temp;
                                                HarBri.rf_des_kd=kd_temp;HarBri.rm_des_kd=kd_temp;HarBri.rb_des_kd=kd_temp;

                                                HarBri.lfDataLoading(HarBri.lf_des_pos, HarBri.lf_des_vel, HarBri.lf_des_tor, HarBri.lf_des_kp, HarBri.lf_des_kd);
                                                HarBri.lmDataLoading(HarBri.lm_des_pos, HarBri.lm_des_vel, HarBri.lm_des_tor, HarBri.lm_des_kp, HarBri.lm_des_kd);
                                                HarBri.lbDataLoading(HarBri.lb_des_pos, HarBri.lb_des_vel, HarBri.lb_des_tor, HarBri.lb_des_kp, HarBri.lb_des_kd);
                                                HarBri.rfDataLoading(HarBri.rf_des_pos, HarBri.rf_des_vel, HarBri.rf_des_tor, HarBri.rf_des_kp, HarBri.rf_des_kd);
                                                HarBri.rmDataLoading(HarBri.rm_des_pos, HarBri.rm_des_vel, HarBri.rm_des_tor, HarBri.rm_des_kp, HarBri.rm_des_kd);
                                                HarBri.rbDataLoading(HarBri.rb_des_pos, HarBri.rb_des_vel, HarBri.rb_des_tor, HarBri.rb_des_kp, HarBri.rb_des_kd);
                                                KEYBOARD_CONTINUE_MODE=0; //lcc : =0 表示只发送一次
                                        }
                                        break; 
                                case '-':     //lcc 20230506:当前角度－１
                                        {
                                                Eigen::Vector3d one;
                                                one.setOnes();
      
                                                HarBri.lf_des_pos=leg_root.joint_rec_pos.block<1,3>(0,0).transpose()-one*1*HarBri._RAD1; 
                                                HarBri.lm_des_pos=leg_root.joint_rec_pos.block<1,3>(0,3).transpose()-one*1*HarBri._RAD1; 
                                                HarBri.lb_des_pos=leg_root.joint_rec_pos.block<1,3>(0,6).transpose()-one*1*HarBri._RAD1; 
                                                HarBri.rf_des_pos=leg_root.joint_rec_pos.block<1,3>(0,9).transpose()-one*1*HarBri._RAD1; 
                                                HarBri.rm_des_pos=leg_root.joint_rec_pos.block<1,3>(0,12).transpose()-one*1*HarBri._RAD1; 
                                                HarBri.rb_des_pos=leg_root.joint_rec_pos.block<1,3>(0,15).transpose()-one*1*HarBri._RAD1; 

                                                HarBri.lf_des_tor.setZero();HarBri.lm_des_tor.setZero(); HarBri.lb_des_tor.setZero();
                                                HarBri.rf_des_tor.setZero();HarBri.rm_des_tor.setZero();HarBri.rb_des_tor.setZero();

                                                HarBri.lf_des_vel.setZero();HarBri.lm_des_vel.setZero();HarBri.lb_des_vel.setZero();
                                                HarBri.rf_des_vel.setZero();HarBri.rm_des_vel.setZero();HarBri.rb_des_vel.setZero();
                                                
                                                Eigen::Vector3d kp_temp, kd_temp;
                                                kp_temp<< 80, 80, 80;
                                                kd_temp<< 3, 3, 3;
                                                // kp_temp<< 20, 20, 20;
                                                // kd_temp<< 2, 2, 2;
                                                // kp_temp<< 10, 10, 10;
                                                // kd_temp<< 1, 1, 1;
                                                HarBri.lf_des_kp=kp_temp;HarBri.lm_des_kp=kp_temp;HarBri.lb_des_kp=kp_temp;
                                                HarBri.rf_des_kp=kp_temp;HarBri.rm_des_kp=kp_temp;HarBri.rb_des_kp=kp_temp;

                                                HarBri.lf_des_kd=kd_temp;HarBri.lm_des_kd=kd_temp;HarBri.lb_des_kd=kd_temp;
                                                HarBri.rf_des_kd=kd_temp;HarBri.rm_des_kd=kd_temp;HarBri.rb_des_kd=kd_temp;

                                                HarBri.lfDataLoading(HarBri.lf_des_pos, HarBri.lf_des_vel, HarBri.lf_des_tor, HarBri.lf_des_kp, HarBri.lf_des_kd);
                                                HarBri.lmDataLoading(HarBri.lm_des_pos, HarBri.lm_des_vel, HarBri.lm_des_tor, HarBri.lm_des_kp, HarBri.lm_des_kd);
                                                HarBri.lbDataLoading(HarBri.lb_des_pos, HarBri.lb_des_vel, HarBri.lb_des_tor, HarBri.lb_des_kp, HarBri.lb_des_kd);
                                                HarBri.rfDataLoading(HarBri.rf_des_pos, HarBri.rf_des_vel, HarBri.rf_des_tor, HarBri.rf_des_kp, HarBri.rf_des_kd);
                                                HarBri.rmDataLoading(HarBri.rm_des_pos, HarBri.rm_des_vel, HarBri.rm_des_tor, HarBri.rm_des_kp, HarBri.rm_des_kd);
                                                HarBri.rbDataLoading(HarBri.rb_des_pos, HarBri.rb_des_vel, HarBri.rb_des_tor, HarBri.rb_des_kp, HarBri.rb_des_kd);
                                                KEYBOARD_CONTINUE_MODE=0; //lcc : =0 表示只发送一次
                                        }
                                        break; 
                                case 'i':     //lcc 20230506:进入闭环
                                        {
                                                HarBri.robDataLoadEnterCloseLoop();
                                                KEYBOARD_CONTINUE_MODE=1;
                                        }
                                        break; 
                                case 'o':    //lcc 20230506:退出闭环
                                        {
                                                HarBri.robDataLoadExitCloesLoop();
                                                KEYBOARD_CONTINUE_MODE=1;
                                        }
                                        break;
                                case 's':    //lcc 20230506：切换发送模式为线程发送，开启６个发送线程
                                        {       mtx10.lock();
                                                HardwareBridge::global_send_thread_flag=1;
                                                HarBri.openSendThread(1);
                                                switch_flag=0;
                                                mtx10.unlock();
                                                KEYBOARD_CONTINUE_MODE=0;
                                        }
                                        break;
                                case 'p':    //lcc 20230330:进入算法程序
                                        {
                                                if(HarBri.lf_protect_init_flag==1 && HarBri.lm_protect_init_flag==1 && HarBri.lb_protect_init_flag==1 &&
                                                   HarBri.rf_protect_init_flag==1 && HarBri.rm_protect_init_flag==1 && HarBri.rb_protect_init_flag==1 )
                                                {
                                                        break_flag=0;
                                                        printf(" motors ready\n");
                                                }
                                                else
                                                        printf(" motors no ready\n");
                                                KEYBOARD_CONTINUE_MODE=1;
                                                
                                        }
                                        break;
                                default: ;
                                        break;
                        } 

                        /**
                        * @brief 接受机器人在所有数据，并且更新时间序列
                        */
                        HarBri.dataRecAllAndSeqUpdate();

                        /**
                        * @brief 将机器人的所有数据都放到1x18在数组里
                        */
                        HarBri.dataIntegrat();
                        /**
                        * @brief 接收gazebo或机器人在数据，存储在 joi_act_vel，joi_act_tor中
                        */
                        recData();

                        // HarBri.showAllData();
                        // cout << "cost time:" << _Tim1.getTimerMilliSec()-1 <<"ms     "<<endl;
                        // dt_ms=_Tim1.getTimerSecond();
                        // dt_s=_Tim1.getTimerSecond();
                        // _Tim1.update();
                        if(HarBri.output_sequnce!=output_sequnce_last)
                        {
                                cout << "cost time:" << _Tim1.getTimerMilliSec()-1 <<"ms     "<<HarBri.output_sequnce<<endl;
                                dt_ms=_Tim1.getTimerSecond();
                                dt_s=_Tim1.getTimerSecond();
                                _Tim1.update();
                                output_sequnce_last=HarBri.output_sequnce;
                        }

                        if(switch_flag==1)
                        {
                                HarBri.showAllData();
                                HarBri.lf_des_pos=HarBri.lf_act_pos;
                                HarBri.lm_des_pos=HarBri.lm_act_pos;
                                HarBri.lb_des_pos=HarBri.lb_act_pos;
                                HarBri.rf_des_pos=HarBri.rf_act_pos;
                                HarBri.rm_des_pos=HarBri.rm_act_pos;
                                HarBri.rb_des_pos=HarBri.rb_act_pos;
                                HarBri.sendAllLegMsg();
                        }
                        // else
                        // {
                        //         printf("protect_init_flag: %d  %d  %d  %d  %d  %d  \n"
                        //         ,HarBri.lf_protect_init_flag, HarBri.lm_protect_init_flag, HarBri.lb_protect_init_flag
                        //         , HarBri.rf_protect_init_flag, HarBri.rm_protect_init_flag, HarBri.rb_protect_init_flag);
                        // }

                        // usleep(10000);
                }
        #elif HARD_WARE==2
                GazeboSim.legGazeboSimInit(argc,argv);
                pubMsgTopicName ImuLinAcc("ImuLinAcc");
                pubMsgTopicName ImuAngVel("ImuAngVel");

                pubMsgTopicName LfForce("LfForce");
                pubMsgTopicName LmForce("LmForce");
                pubMsgTopicName LbForce("LbForce");
                pubMsgTopicName RfForce("RfForce");
                pubMsgTopicName RmForce("RmForce");
                pubMsgTopicName RbForce("RbForce");

                pubMsgTopicName LfVel("LfVel");
                pubMsgTopicName LmVel("LmVel");
                pubMsgTopicName LbVel("LbVel");
                pubMsgTopicName RfVel("RfVel");
                pubMsgTopicName RmVel("RmVel");
                pubMsgTopicName RbVel("RbVel");

                pubMsgTopicName ang_vel("ang_vel");
                pubMsgTopicName lin_acc("lin_acc");
                pubMsgTopicName eulerAngle("eulerAngle");

                pubMsgTopicName staest_vel("staest_vel");
                pubMsgTopicName staest_pos("staest_pos");

                //xzb230512
                pubMsgTopicName vel_legodom_pub("vel_legodom");
                pubMsgTopicName vel_ekf_pub("vel_ekf");
                pubMsgTopicName pos_ekf_pub("pos_ekf");

                // ros::Rate rate(250);
        #endif  

        while(1)
        {       
                signal(SIGINT, MySigintHandlera );//lcc 20230329: 所使用快捷键ctrl+c，中断程序运行。
                printf("");
                /**
                * @details 设置机器人运动参数，接受键盘控制机器人运动
                */
                parSeting(); 
                cpg_scheduler=_Cpg.cpgNewRun();
                // std::cout<<"cpg_scheduler"<<std::endl;
                // std::cout<<cpg_scheduler<<std::endl;

                /**
                * @details 1.最终基于轨迹得到各电机的期望角度joi_des_pos; 
                * @details 2.机器人启动时，会根据当前电机真实角度进入启动姿态; 
                * @details 3.通过sendDataConPro()对joi_des_pos的连续性进行保护; 
                */
                trajectoryPlaning(); 

                //lcc 20230329: 决定控制硬件还是运行仿真
                #if HARD_WARE==1  //lcc 20230329: 开启硬件，以下是发送数据得程序
                        // lcc 20230323：放开下行备注，会打印每次运行点时间
                        // HarBri.showAllData();
                        dt_ms=_Tim1.getTimerMilliSec();
                        // cout << "cost time:" << dt_ms <<"ms     "<< "rec_data_seq" << rec_data_seq << endl;
                        dt_s=_Tim1.getTimerSecond();
                        _Tim1.update();
                        /**
                        * @details 1.在recData()中接收到真实关节位置后，get_joint_pos_flag=true后,
                        *               joint_pos_run_count会在trajectoryPlaning()中进行累加;
                        * @details 2.通过velLimAnddifFroDesPosAndActPos()对joi_act_vel、joi_act_pos与joi_des_pos在差值进行保护; 
                        * @details 3.最后调用realHexMsgLoadAndSend()来设置kpkd发送电机的控制; 
                        */
                        realRobMsgPub();
                        if(dt_ms<=4.0)
                        {
                                usleep( (4.0-dt_ms)*1000 );
                                // printf(" 4.0-dt_ms:%f \n",4.0-dt_ms);
                        }
                #elif HARD_WARE==2  //lcc 20230329: 开启仿真
                        dt_ms=_Tim1.getTimerMilliSec();
                        dt_s=_Tim1.getTimerSecond();
                        _Tim1.update();
                        // printf("000000\n");
                        #if SIM_CTRL_MODE==1  //lcc 20230329: 切换机器人在仿真中的控制方式
                                if(joint_pos_run_count>=3) positionController();
                        #elif SIM_CTRL_MODE==2
                                if(joint_pos_run_count>=3) pdController();
                                // if(joint_pos_run_count>=3) adaController();
                        #endif
                        // printf("0a0a0aa\n");
                        //lcc 20230329: 以下是pub的程序
                        simMsgPub();
                        if(dt_ms<=3)
                        {
                                usleep( (3-dt_ms)*1000 );
                                // printf(" 3-dt_ms:%f \n",3-dt_ms);
                        }
                        // LfForce.msgPubRun(_Lf.m.robot_velocity);
                        // LmForce.msgPubRun(_Lm.m.robot_velocity);
                        // LbForce.msgPubRun(_Lb.m.robot_velocity);
                        // RfForce.msgPubRun(_Rf.m.robot_velocity);
                        // RmForce.msgPubRun(_Rm.m.robot_velocity);
                        // RbForce.msgPubRun(_Rb.m.robot_velocity);
                        // LfVel.msgPubRun(_Lf.m.robot_force);
                        // LmVel.msgPubRun(_Lm.m.robot_force);
                        // LbVel.msgPubRun(_Lb.m.robot_force);
                        // RfVel.msgPubRun(_Rf.m.robot_force);
                        // RmVel.msgPubRun(_Rm.m.robot_force);
                        // RbVel.msgPubRun(_Rb.m.robot_force);
                        ang_vel.msgPubRun(body_root.imu_ang_vel);
                        lin_acc.msgPubRun(body_root.imu_lin_acc);
                        eulerAngle.msgPubRun(body_root.eulerAngle);

                        staest_vel.msgPubRun(state.root_lin_vel);
                        staest_pos.msgPubRun(state.root_pos);

                        //xzb230512
                        vel_legodom_pub.msgPubRun(vel_legodom);
                        vel_ekf_pub.msgPubRun(vel_ekf);
                        pos_ekf_pub.msgPubRun(pos_ekf);
                #endif

                if(start_thread_flag)
                {       
                        start_thread_flag=false;
                        #if HARD_WARE==1
                                // HarBri.robDataLoadExitCloesLoop();
                        #elif HARD_WARE==2
                                GazeboSim.legMsgSUb();
                                GazeboSim.imuMsgSUb();
                                #if SEQ_CHOKE==1
                                        pthread_t TopicLisenerThr;
                                        pthread_create(&TopicLisenerThr,NULL,TopicLisener,NULL);
                                #endif
                        #endif
                }

                /**
                * @details 1.接收关节信息
                * @details 2.基于时间序列是否更新来阻塞，这个功能可根据宏SEQ_CHOKE来开启or关闭
                * @details 3.调用recJointData()接收gazebo或机器人在数据，存储在 joi_act_vel，joi_act_tor中; 
                * @details 4.判断是否接收到关节信息，标志位存在get_joint_pos_flag中; 若是真实机器人，会限制上电位置;
                * @details 5．接收其他传感消息;
                * @author lcc
                */
                recData();     
                recDataHandling();  //lcc 20230329: 对得到的数据进行转换和处理
                // msgShow(); //lcc 20230329: 打印机器人的信息数

                // _LagrangeInterpolator.eg_bytime( _SimpleScheduler.retSimpleScheduler(1, 0.01) );

                if(motor_pos_ach_count_flag==1 ) // motor_pos_ach_count_flag==1表示机器人已经完成启动动作
                {
                        if( _set_static_pos_conver[0].retConvDoneFlag() && 
                        _set_static_pos_conver[1].retConvDoneFlag() && 
                        _set_static_pos_conver[2].retConvDoneFlag() && 
                        _set_static_pos_conver[3].retConvDoneFlag() && 
                        _set_static_pos_conver[4].retConvDoneFlag() && 
                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag 
                        )
                        {              
                                #if ADAPTIV_FLAG==1
                                        int movement_mode=0;
                                        movement_mode=KeyBoardCtrl.retKeyValue_movemode();
                                        if(movement_mode=='w' || movement_mode=='q' || movement_mode=='e' ||   //lcc　表示机器人运动模式
                                                movement_mode=='a' || movement_mode=='s' || movement_mode=='d' )
                                        {

                                                #if ONLY_Quadruped==1  //机器人不不可切换步态，仅仅可用四足步态
                                                                //lcc 20230513: 腿默认顺序为: lf lm lb rf rm rb -> 0 1 2 3 4 5
                                                                cpg_touch_down_scheduler<<cpg_scheduler(1,4), cpg_scheduler(1,3),
                                                                                        cpg_scheduler(1,5), cpg_scheduler(1,2), cpg_scheduler(1,1), cpg_scheduler(1,0);
                                                #elif ONLY_Quadruped==0
                                                                //lcc 20230513: 腿默认顺序为: lf lm lb rf rm rb -> 0 1 2 3 4 5
                                                                cpg_touch_down_scheduler<<cpg_scheduler(1,3), cpg_scheduler(1,4),
                                                                                        cpg_scheduler(1,5), cpg_scheduler(1,2), cpg_scheduler(1,1), cpg_scheduler(1,0);
                                                #endif
                                        }
                                        else if(movement_mode=='c')  //lcc　机器人蹲
                                                cpg_touch_down_scheduler<< 404, 404, 404, 404, 404, 404;
                                        else                        //lcc　机器人站立，六腿着地
                                                cpg_touch_down_scheduler<< 0, 0, 0, 0, 0, 0;  //lcc 0 表示cpg_scheduler触地
                                                // cpg_touch_down_scheduler<< 404, 404, 404, 404, 404, 404;
                                        ContactSimple.swingphase_contact_est(leg_root.foot_ret_force, cpg_touch_down_scheduler, leg_root.foot_swing_traj);
                                #endif

                                if(StateEstimator.state_estimator_start_flag==true)
                                {
                                        // xzb230512 add 临时使用cpgy信号替代触地检测——实体机器人上要删去
                                        Eigen::Matrix<double,num_leg*3,1> emty_grf,Pfoot_b,Vfoot_v;      emty_grf.setZero();
                                        for (int i_leg = 0; i_leg < 6; i_leg++)
                                        {
                                                Pfoot_b.block(3*i_leg,0,3,1)=leg_root.foot_act_force.col(i_leg);
                                                Vfoot_v.block(3*i_leg,0,3,1)=body_root.foot_act_vel.col(i_leg);
                                        }
                                        
                                        // cout<<"ready get data for legodom"<<endl;
                                        _XzbEstimator.legOdom_cpgy( cpg_touch_down_scheduler);
                                        _XzbEstimator.legOdom_EKF(body_root.imu_lin_acc,
                                                body_root.imu_ang_vel,
                                                emty_grf,
                                                emty_grf,
                                                Pfoot_b,Vfoot_v
                                                );
                                        vel_legodom=_XzbEstimator.COMvelocity_touchpredict; 
                                        vel_ekf=_XzbEstimator.bodystatedata._vCOM;
                                        pos_ekf=_XzbEstimator.bodystatedata._pCOM;
                                        rpy_ekf=_XzbEstimator.bodystatedata._rpyCOM;
   
                                        if(_state_estimator_flag==true)  //lcc 20230504:状态估计  ok
                                        {       
                                                printf("motor_pos_ach_count_flag:%d\n",motor_pos_ach_count_flag);
                                                StateEstimator.init_state(body_root.foot_act_vel,body_root.root_rot_mat);//lcc 20230428
                                                _state_estimator_flag=false;
                                        }
                                        else if(_state_estimator_flag==false)
                                        {
                                                StateEstimator.update_estimation
                                                (
                                                        body_root.imu_ang_vel,
                                                        body_root.imu_lin_acc,
                                                        body_root.root_rot_mat,
                                                        body_root.foot_act_pos ,
                                                        body_root.foot_act_vel,
                                                        dt_s,
                                                        KeyBoardCtrl.retKeyValue_movemode(), //lcc 20230428:int movement_mode->运动模式目前是跟据键盘来控制的
                                                        cpg_touch_down_scheduler
                                                );  
                                                // lcc :取出状态估计的结果
                                                // _state_estimator_mut.lock();
                                                state.root_pos=StateEstimator.state.estimated_root_pos;
                                                state.root_lin_vel=StateEstimator.state.estimated_root_lin_vel;
                                                state.foot_pos_ref_world=StateEstimator.state.estimated_foot_pos_ref_world;  
                                                // _state_estimator_mut.unlock();
                                        }            
                                }
                        }
                }
 
                #if TXT_FLAGE  
                        if ( ! robot_postion) { cout << "文件不能打开" <<endl; }
                        robot_postion <<  
                        leg_root.foot_act_pos(0,5) <<" "<< 
                        leg_root.foot_act_pos(1,5) <<" "<< 
                        leg_root.foot_act_pos(2,5) << endl;

                        if ( ! robot_velocity) { cout << "文件不能打开" <<endl; }
                        robot_velocity << 
                        body_root.foot_act_vel(0,5) <<" "<< 
                        body_root.foot_act_vel(1,5) <<" "<< 
                        body_root.foot_act_vel(2,5) << endl;
                       
                        // if ( ! robot_force) { cout << "文件不能打开" <<endl; }
                        // robot_force << 
                        // leg_root.joint_rec_vel(0) <<" "<< 
                        // leg_root.joint_rec_vel(1) <<" "<< 
                        // leg_root.joint_rec_vel(2) <<" "<< 
                        // leg_root.joint_rec_vel(3) <<" "<< 
                        // leg_root.joint_rec_vel(4) <<" "<< 
                        // leg_root.joint_rec_vel(5) <<" "<< 
                        // leg_root.joint_rec_vel(6) <<" "<< 
                        // leg_root.joint_rec_vel(7) <<" "<< 
                        // leg_root.joint_rec_vel(8) <<" "<< 
                        // leg_root.joint_rec_vel(9) <<" "<< 
                        // leg_root.joint_rec_vel(10) <<" "<< 
                        // leg_root.joint_rec_vel(11) <<" "<< 
                        // leg_root.joint_rec_vel(12) <<" "<< 
                        // leg_root.joint_rec_vel(13) <<" "<< 
                        // leg_root.joint_rec_vel(14) <<" "<< 
                        // leg_root.joint_rec_vel(15) <<" "<< 
                        // leg_root.joint_rec_vel(16) <<" "<< 
                        // leg_root.joint_rec_vel(17) << endl;
                        
                        // if ( ! robot_velocity) { cout << "文件不能打开" <<endl; }
                        // robot_velocity << 
                        // leg_root.joint_rec_tor(0) <<" "<< 
                        // leg_root.joint_rec_tor(1) <<" "<< 
                        // leg_root.joint_rec_tor(2) <<" "<< 
                        // leg_root.joint_rec_tor(3) <<" "<< 
                        // leg_root.joint_rec_tor(4) <<" "<< 
                        // leg_root.joint_rec_tor(5) <<" "<< 
                        // leg_root.joint_rec_tor(6) <<" "<< 
                        // leg_root.joint_rec_tor(7) <<" "<< 
                        // leg_root.joint_rec_tor(8) <<" "<< 
                        // leg_root.joint_rec_tor(9) <<" "<< 
                        // leg_root.joint_rec_tor(10) <<" "<< 
                        // leg_root.joint_rec_tor(11) <<" "<< 
                        // leg_root.joint_rec_tor(12) <<" "<< 
                        // leg_root.joint_rec_tor(13) <<" "<< 
                        // leg_root.joint_rec_tor(14) <<" "<< 
                        // leg_root.joint_rec_tor(15) <<" "<< 
                        // leg_root.joint_rec_tor(16) <<" "<< 
                        // leg_root.joint_rec_tor(17) << endl;

                        // if ( ! robot_postion) { cout << "文件不能打开" <<endl; }
                        // robot_postion << 
                        // leg_root.joint_rec_pos(0) <<" "<< 
                        // leg_root.joint_rec_pos(1) <<" "<< 
                        // leg_root.joint_rec_pos(2) <<" "<< 
                        // leg_root.joint_rec_pos(3) <<" "<< 
                        // leg_root.joint_rec_pos(4) <<" "<< 
                        // leg_root.joint_rec_pos(5) <<" "<< 
                        // leg_root.joint_rec_pos(6) <<" "<< 
                        // leg_root.joint_rec_pos(7) <<" "<< 
                        // leg_root.joint_rec_pos(8) <<" "<< 
                        // leg_root.joint_rec_pos(9) <<" "<< 
                        // leg_root.joint_rec_pos(10) <<" "<< 
                        // leg_root.joint_rec_pos(11) <<" "<< 
                        // leg_root.joint_rec_pos(12) <<" "<< 
                        // leg_root.joint_rec_pos(13) <<" "<< 
                        // leg_root.joint_rec_pos(14) <<" "<< 
                        // leg_root.joint_rec_pos(15) <<" "<< 
                        // leg_root.joint_rec_pos(16) <<" "<< 
                        // leg_root.joint_rec_pos(17) << endl;

                #endif
                // printf("--------next-------------\n");
        }
}

/**
* @details 设置机器人运动参数，接受键盘控制机器人运动
*/
void Hexapod::parSeting(void)
{
        if(start_thread_flag)  //lcc 20230330:仅仅在机器人启动时会进入，robStand(1)表上一个时间点机器人就得到
                robSquat(1);   //lcc 20230330:robSquat(1)表示一个时间点机器人就得到了要蹲下姿态所需点所有电机的角度

        #if HARD_WARE==1
                _Cpg.control_cycle=_Cpg.CtrlCyclLinTran.linearConvert(_Cpg.control_cycle,set_cpg_ctrl_cycle,180); //lcc 20230418:设计一个周期点数
                _Cpg.T=_Cpg.TLinTran.linearConvert(_Cpg.T,1,180);
        #elif HARD_WARE==2
                _Cpg.control_cycle=_Cpg.CtrlCyclLinTran.linearConvert(_Cpg.control_cycle,set_cpg_ctrl_cycle,180); //lcc 20230418:设计一个周期点数  一个步态周期的点数＝１／set_cpg_ctrl_cycle
        #endif
        //  std::cout<<_Cpg.control_cycle<<std::endl;

        keyBoardControl(KeyBoardCtrl.retKeyValue()); //lcc 20230409:跟据键盘指令，控制机器人

        if(set_para_init_flag==1)   //lcc 初始设置步长和步高的ｋ值
        {
                set_x_deviation=0; set_y_deviation=0; set_z_deviation=0; 
                set_yaw=0; set_roll=0; set_pitch=0;
                set_step_length_k=1.1;
                set_step_hight_k=1.3;
                set_para_init_flag=0;
                #if HARD_WARE==1
                        set_cpg_ctrl_cycle=0.0025;
                #elif HARD_WARE==2
                        set_cpg_ctrl_cycle=0.0015;   //lcc 一个步态周期的点数＝１／set_cpg_ctrl_cycle
                #endif
                printf("init\n");

                cpg_touch_down_scheduler<< 404, 404, 404, 404, 404, 404;
        }
        
        neur_bezier[0].length_k=_step_length_k_conver[0].linearConvert(neur_bezier[0].length_k,set_step_length_k,60);
        neur_bezier[1].length_k=_step_length_k_conver[1].linearConvert(neur_bezier[1].length_k,set_step_length_k,60);
        neur_bezier[2].length_k=_step_length_k_conver[2].linearConvert(neur_bezier[2].length_k,set_step_length_k,60);
        neur_bezier[3].length_k=_step_length_k_conver[3].linearConvert(neur_bezier[3].length_k,set_step_length_k,60);
        neur_bezier[4].length_k=_step_length_k_conver[4].linearConvert(neur_bezier[4].length_k,set_step_length_k,60);
        neur_bezier[5].length_k=_step_length_k_conver[5].linearConvert(neur_bezier[5].length_k,set_step_length_k,60);

        neur_bezier[0].height_k=_step_hight_k_conver[0].linearConvert(neur_bezier[0].height_k,set_step_hight_k,60);
        neur_bezier[1].height_k=_step_hight_k_conver[1].linearConvert(neur_bezier[1].height_k,set_step_hight_k,60);
        neur_bezier[2].height_k=_step_hight_k_conver[2].linearConvert(neur_bezier[2].height_k,set_step_hight_k,60);
        neur_bezier[3].height_k=_step_hight_k_conver[3].linearConvert(neur_bezier[3].height_k,set_step_hight_k,60);
        neur_bezier[4].height_k=_step_hight_k_conver[4].linearConvert(neur_bezier[4].height_k,set_step_hight_k,60);
        neur_bezier[5].height_k=_step_hight_k_conver[5].linearConvert(neur_bezier[5].height_k,set_step_hight_k,60);

        // std::cout<<"cpg_touch_down_scheduler"<<std::endl;
        // std::cout<<cpg_touch_down_scheduler<<std::endl;
        if(t_test_key==1 )
        {       
                // 姿态控制,pid实现闭环姿态控制.
                double roll, pitch;

                // -- 以下是x轴转角的调整 --//
                //lcc 20230531:set_roll极其不稳定，不可用
                // if(-body_root.eulerAngle(2)*_RAD2>=10) roll=10*_RAD1;
                // else if(-body_root.eulerAngle(2)*_RAD2<=-10) roll=-10*_RAD1;
                // else    roll=-body_root.eulerAngle(2);

                // attitude_pid_ctrl[2].setKpKiKd(3, 0.003, 0.3);
                // set_roll=attitude_pid_ctrl[2].positonPidTau( 0.0, -body_root.eulerAngle(2) );

                // if(set_roll>=35) set_roll=35;
                // else if(set_roll<=-35) set_roll=-35;

                // -- 以下是y轴转角的调整 --//
                if(-body_root.eulerAngle(1)*_RAD2>=10) pitch=10*_RAD1;
                else if(-body_root.eulerAngle(1)*_RAD2<=-10) pitch=-10*_RAD1;
                else    pitch=-body_root.eulerAngle(1);

                attitude_pid_ctrl[1].setKpKiKd(2, 0.002, 0.2);
                set_pitch=attitude_pid_ctrl[1].positonPidTau( 0.0, -body_root.eulerAngle(1) );

                // if(set_pitch>=35) set_pitch=35;
                // else if(set_pitch<=-35) set_pitch=-35;

                for(int i=0; i<6; i++)
                {
                        // if( cpg_touch_down_scheduler(i)==0 )  // cpg_touch_down_scheduler = 0 表示cpg_scheduler触地; 触地腿才可以参与姿态控制
                        {
                                _FooBodAdjMap[i].rrr_yaw=_FooBodAdjMap[i].YawLinTran.linearConvert(_FooBodAdjMap[i].rrr_yaw,set_yaw,1);
                                _FooBodAdjMap[i].rrr_roll=_FooBodAdjMap[i].RolLinTran.linearConvert(_FooBodAdjMap[i].rrr_roll,set_roll,1);
                                _FooBodAdjMap[i].rrr_pitch=_FooBodAdjMap[i].PitLinTran.linearConvert(_FooBodAdjMap[i].rrr_pitch,set_pitch,1);
                                _FooBodAdjMap[i].fuselageAttiuCtrl(set_yaw, set_roll, set_pitch);

                                _FooBodAdjMap[i].x_deviation=_FooBodAdjMap[i].XdeLinTran.linearConvert(_FooBodAdjMap[i].x_deviation,set_x_deviation,1);
                                _FooBodAdjMap[i].y_deviation=_FooBodAdjMap[i].YdeLinTran.linearConvert(_FooBodAdjMap[i].y_deviation,set_y_deviation,1);
                                _FooBodAdjMap[i].z_deviation=_FooBodAdjMap[i].ZdeLinTran.linearConvert(_FooBodAdjMap[i].z_deviation,set_z_deviation,1);
                                _FooBodAdjMap[i].fuselageDeviCtrl(set_x_deviation,set_y_deviation,set_z_deviation);    
                        }
                }
                // FooTipAndBodAdjMap::fuselageAttiuCtrl(set_yaw, set_roll, set_pitch);
                // FooTipAndBodAdjMap::fuselageDeviCtrl(set_x_deviation, set_y_deviation, set_z_deviation);
        }
        else if(t_test_key==0)
        {
                //lcc 这一块是手动控制模式，所有有姿态的调整接口;上面那一块是自适应控制模式;
                //  lcc 20230330:机器人yaw,roll,pitch姿态的调整
                for(int i=0; i<6; i++)
                {
                        _FooBodAdjMap[i].rrr_yaw=_FooBodAdjMap[i].YawLinTran.linearConvert(_FooBodAdjMap[i].rrr_yaw,set_yaw,120);
                        _FooBodAdjMap[i].rrr_roll=_FooBodAdjMap[i].RolLinTran.linearConvert(_FooBodAdjMap[i].rrr_roll,set_roll,120);
                        _FooBodAdjMap[i].rrr_pitch=_FooBodAdjMap[i].PitLinTran.linearConvert(_FooBodAdjMap[i].rrr_pitch,set_pitch,120);
                        _FooBodAdjMap[i].fuselageAttiuCtrl(_FooBodAdjMap[i].rrr_yaw,_FooBodAdjMap[i].rrr_roll,_FooBodAdjMap[i].rrr_pitch);

                        _FooBodAdjMap[i].x_deviation=_FooBodAdjMap[i].XdeLinTran.linearConvert(_FooBodAdjMap[i].x_deviation,set_x_deviation,120);
                        _FooBodAdjMap[i].y_deviation=_FooBodAdjMap[i].YdeLinTran.linearConvert(_FooBodAdjMap[i].y_deviation,set_y_deviation,120);
                        _FooBodAdjMap[i].z_deviation=_FooBodAdjMap[i].ZdeLinTran.linearConvert(_FooBodAdjMap[i].z_deviation,set_z_deviation,120);
                        _FooBodAdjMap[i].fuselageDeviCtrl(_FooBodAdjMap[i].x_deviation,_FooBodAdjMap[i].y_deviation,_FooBodAdjMap[i].z_deviation);
                }
        }

        std::cout<<"set_z_deviation"<<std::endl;
        std::cout<<set_z_deviation<<std::endl;

        // std::cout<<"set_pitch*_RAD2"<<std::endl;
        // std::cout<<set_pitch*_RAD2<<std::endl;
        // std::cout<<"body_root.eulerAngle(1)*_RAD2"<<std::endl;
        // std::cout<<body_root.eulerAngle(1)*_RAD2<<std::endl;

        // std::cout<<"set_roll*_RAD2"<<std::endl;
        // std::cout<<set_roll*_RAD2<<std::endl;
        // std::cout<<"body_root.eulerAngle(2)*_RAD2"<<std::endl;
        // std::cout<<body_root.eulerAngle(2)*_RAD2<<std::endl;

        // std::cout<<"--------next aa------"<<std::endl;
}



