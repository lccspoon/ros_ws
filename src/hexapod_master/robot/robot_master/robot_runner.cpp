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
#include <Eigen/Core>
#include <Eigen/Dense>

//lcc 20230329:一些类中静态全局变量的赋初始值
double FooTipAndBodAdjMap::RRR_YAW=0;double FooTipAndBodAdjMap::RRR_ROLL=0;double FooTipAndBodAdjMap::RRR_PITCH=0;
double FooTipAndBodAdjMap::X_DEVIATION=0;double FooTipAndBodAdjMap::Y_DEVIATION=0;double FooTipAndBodAdjMap::Z_DEVIATION=0;
double kinematic::LEG_DH_PARAM1=0;double kinematic::LEG_DH_PARAM2=0;double kinematic::LEG_DH_PARAM3=0;

//xzb230512  全局变量用于存储需要发布的话题信息
Eigen::Vector3d vel_legodom,vel_ekf,rpy_ekf,pos_ekf;
//20230519 
extern Eigen::Matrix<double,18,1>invDesangle,invDesangle;

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
                ofstream foo_act_vel_ref_body,foo_act_for,foo_tip_act_pos_ref_leg_ori;
                foo_act_vel_ref_body.open("./foo_act_vel_ref_body.txt", ios::out | ios::trunc);
                foo_act_for.open("./foo_act_for.txt", ios::out | ios::trunc);
                foo_tip_act_pos_ref_leg_ori.open("./foo_tip_act_pos_ref_leg_ori.txt", ios::out | ios::trunc);
        #endif

        #if HARD_WARE==1
        
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

                                                HarBri.lf_des_tor.setZero();
                                                HarBri.lm_des_tor.setZero();
                                                HarBri.lb_des_tor.setZero();
                                                HarBri.rf_des_tor.setZero();
                                                HarBri.rm_des_tor.setZero();
                                                HarBri.rb_des_tor.setZero();

                                                HarBri.lf_des_vel.setZero();
                                                HarBri.lm_des_vel.setZero();
                                                HarBri.lb_des_vel.setZero();
                                                HarBri.rf_des_vel.setZero();
                                                HarBri.rm_des_vel.setZero();
                                                HarBri.rb_des_vel.setZero();
                                                
                                                Eigen::Vector3d kp_temp, kd_temp;
                                                // kp_temp<< 80, 80, 80;
                                                // kd_temp<< 3, 3, 3;
                                                kp_temp<< 20, 20, 20;
                                                kd_temp<< 2, 2, 2;
                                                HarBri.lf_des_kp=kp_temp;
                                                HarBri.lm_des_kp=kp_temp;
                                                HarBri.lb_des_kp=kp_temp;
                                                HarBri.rf_des_kp=kp_temp;
                                                HarBri.rm_des_kp=kp_temp;
                                                HarBri.rb_des_kp=kp_temp;

                                                HarBri.lf_des_kd=kd_temp;
                                                HarBri.lm_des_kd=kd_temp;
                                                HarBri.lb_des_kd=kd_temp;
                                                HarBri.rf_des_kd=kd_temp;
                                                HarBri.rm_des_kd=kd_temp;
                                                HarBri.rb_des_kd=kd_temp;

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

                                                HarBri.lf_des_tor.setZero();
                                                HarBri.lm_des_tor.setZero();
                                                HarBri.lb_des_tor.setZero();
                                                HarBri.rf_des_tor.setZero();
                                                HarBri.rm_des_tor.setZero();
                                                HarBri.rb_des_tor.setZero();

                                                HarBri.lf_des_vel.setZero();
                                                HarBri.lm_des_vel.setZero();
                                                HarBri.lb_des_vel.setZero();
                                                HarBri.rf_des_vel.setZero();
                                                HarBri.rm_des_vel.setZero();
                                                HarBri.rb_des_vel.setZero();
                                                
                                                Eigen::Vector3d kp_temp, kd_temp;
                                                // kp_temp<< 80, 80, 80;
                                                // kd_temp<< 3, 3, 3;
                                                kp_temp<< 20, 20, 20;
                                                kd_temp<< 2, 2, 2;
                                                HarBri.lf_des_kp=kp_temp;
                                                HarBri.lm_des_kp=kp_temp;
                                                HarBri.lb_des_kp=kp_temp;
                                                HarBri.rf_des_kp=kp_temp;
                                                HarBri.rm_des_kp=kp_temp;
                                                HarBri.rb_des_kp=kp_temp;

                                                HarBri.lf_des_kd=kd_temp;
                                                HarBri.lm_des_kd=kd_temp;
                                                HarBri.lb_des_kd=kd_temp;
                                                HarBri.rf_des_kd=kd_temp;
                                                HarBri.rm_des_kd=kd_temp;
                                                HarBri.rb_des_kd=kd_temp;

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
                                                // printf("\n---------iiiiiiiiiiiiiii----------------\n");
                                                KEYBOARD_CONTINUE_MODE=1;
                                        }
                                        break; 
                                case 'o':    //lcc 20230506:退出闭环
                                        {
                                                HarBri.robDataLoadExitCloesLoop();
                                                // printf("\n---------ooooooooooooooooo----------------\n");
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
                                                // printf("\n---------ppppppppppppppp----------------\n");
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
                        recJointData();

                        // HarBri.showAllData();
                        // cout << "cost time:" << _Tim1.getTimerMilliSec()-1 <<"ms     "<<endl;
                        dt_ms=_Tim1.getTimerSecond();
                        dt_s=_Tim1.getTimerSecond();
                        _Tim1.update();

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

                        usleep(10000);
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

                //zyt20230519
                pubMsgTopicName invDesangle_pub("invDesangle");
                // pubMsgTopicName LMinvDesangle_pub("LMinvDesangle");
                // pubMsgTopicName LBinvDesangle_pub("LBinvDesangle");
                // pubMsgTopicName RFinvDesangle_pub("RFinvDesangle");
                // pubMsgTopicName RMinvDesangle_pub("RMinvDesangle");
                // pubMsgTopicName RBinvDesangle_pub("RBinvDesangle");
                
                pubMsgTopicName CorrinvDesangle_pub("CorrinvDesangle");                
                // pubMsgTopicName LMCorrinvDesangle_pub("LMCorrinvDesangle");                
                // pubMsgTopicName LBCorrinvDesangle_pub("LBCorrinvDesangle");                
                // pubMsgTopicName RFCorrinvDesangle_pub("RFCorrinvDesangle");                
                // pubMsgTopicName RMCorrinvDesangle_pub("RMCorrinvDesangle");                
                // pubMsgTopicName RBCorrinvDesangle_pub("RBCorrinvDesangle");                


                // ros::Rate rate(250);

        #endif  

        while(1)
        {       

                signal(SIGINT, MySigintHandlera );//lcc 20230329: 所使用快捷键ctrl+c，中断程序运行。

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
                        cout << "cost time:" << _Tim1.getTimerMilliSec()-1 <<"ms     "<< "rec_data_seq" << rec_data_seq << endl;
                        dt_ms=_Tim1.getTimerSecond();
                        dt_s=_Tim1.getTimerSecond();
                        _Tim1.update();
                        /**
                        * @details 1.在recData()中接收到真实关节位置后，get_joint_pos_flag=true后,
                        *               joint_pos_run_count会在trajectoryPlaning()中进行累加;
                        * @details 2.通过velLimAnddifFroDesPosAndActPos()对joi_act_vel、joi_act_pos与joi_des_pos在差值进行保护; 
                        * @details 3.最后调用realHexMsgLoadAndSend()来设置kpkd发送电机的控制; 
                        */
                        realRobMsgPub();
                        if(dt_ms<=8.0)
                        {
                                usleep( (8.0-dt_ms)*1000 );
                                // printf(" 8.0-dt_ms:%f \n",8.0-dt_ms);
                        }
                #elif HARD_WARE==2  //lcc 20230329: 开启仿真
                        dt_ms=_Tim1.getTimerSecond();
                        dt_s=_Tim1.getTimerSecond();
                        _Tim1.update();
                        // printf("000000\n");
                        #if SIM_CTRL_MODE==1  //lcc 20230329: 切换机器人在仿真中的控制方式
                                if(joint_pos_run_count>=3) positionController();
                        #elif SIM_CTRL_MODE==2
                                // if(joint_pos_run_count>=3) pdController();wbcController();
                                if(joint_pos_run_count>=3) adaController();
                        #endif
                        // printf("0a0a0aa\n");
                        //lcc 20230329: 以下是pub的程序
                        simMsgPub();
                        if(dt_ms<=3)
                        {
                                usleep( (3-dt_ms)*1000 );
                                // printf(" 3-dt_ms:%f \n",3-dt_ms);
                        }
                        // LfForce.msgPubRun(_Lf.m.foo_act_for);
                        // LmForce.msgPubRun(_Lm.m.foo_act_for);
                        // LbForce.msgPubRun(_Lb.m.foo_act_for);
                        // RfForce.msgPubRun(_Rf.m.foo_act_for);
                        // RmForce.msgPubRun(_Rm.m.foo_act_for);
                        // RbForce.msgPubRun(_Rb.m.foo_act_for);
                        // LfVel.msgPubRun(_Lf.m.foo_act_vel_ref_body);
                        // LmVel.msgPubRun(_Lm.m.foo_act_vel_ref_body);
                        // LbVel.msgPubRun(_Lb.m.foo_act_vel_ref_body);
                        // RfVel.msgPubRun(_Rf.m.foo_act_vel_ref_body);
                        // RmVel.msgPubRun(_Rm.m.foo_act_vel_ref_body);
                        // RbVel.msgPubRun(_Rb.m.foo_act_vel_ref_body);
                        ang_vel.msgPubRun(body_root.imu_ang_vel);
                        lin_acc.msgPubRun(body_root.imu_lin_acc);
                        eulerAngle.msgPubRun(body_root.eulerAngle);

                        staest_vel.msgPubRun(state.root_lin_vel);
                        staest_pos.msgPubRun(state.root_pos);

                        //xzb230512
                        vel_legodom_pub.msgPubRun(vel_legodom);
                        vel_ekf_pub.msgPubRun(vel_ekf);
                        pos_ekf_pub.msgPubRun(pos_ekf);

                        //zyt20230519
                        CorrinvDesangle_pub.msgPubRun(invDesangle);
                        invDesangle_pub.msgPubRun(invDesangle);

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

                if(motor_pos_ach_count_flag==1 ) // motor_pos_ach_count_flag==1表示机器人已经完成启动动作
                {
                        if( _LfSetStaticStandPosLinTrans.retConvDoneFlag() && 
                        _LmSetStaticStandPosLinTrans.retConvDoneFlag() && 
                        _LbSetStaticStandPosLinTrans.retConvDoneFlag() && 
                        _RfSetStaticStandPosLinTrans.retConvDoneFlag() && 
                        _RmSetStaticStandPosLinTrans.retConvDoneFlag() && 
                        _RbSetStaticStandPosLinTrans.retConvDoneFlag() && stand_done_flag 
                        )
                        {       
                                state.contact_est_scheduler=ContactSimple.contact_estimation(leg_root.foot_ret_force);

                                // std::cout<<"state.contact_est_scheduler"<<std::endl;
                                // std::cout<<state.contact_est_scheduler<<std::endl;
                                // std::cout<<"cpg_touch_down_scheduler"<<std::endl;
                                // std::cout<<cpg_touch_down_scheduler<<std::endl;
                                // std::cout<<"leg_root.foot_ret_force"<<std::endl;
                                // std::cout<<leg_root.foot_ret_force<<std::endl;
                                // std::cout<<"leg_root.foot_swing_traj"<<std::endl;
                                // std::cout<<leg_root.foot_swing_traj<<std::endl;
                                for(int i=0;i<6;i++)
                                {       
                                        Eigen::Vector3d temp;
                                        temp=leg_root.foot_swing_traj.block<3,1>(0,i);
                                        // printf("foot_swing_traj (2):%f  ",temp(2));
                                        if(cpg_touch_down_scheduler(i)==1 && state.contact_est_scheduler(i)==0 && temp(2)>=3*0.01)
                                        {       
                                                // printf("\n");
                                                // printf("cpg_touch_down_scheduler(%d):%f contact_est_scheduler(%d):%f temp(2):%f \n",
                                                // i, cpg_touch_down_scheduler(i), i, state.contact_est_scheduler(i), temp(2));
                                                // _Cpg.cpg_stop_flag=cpg_stop_flag=1;
                                                // printf(" Touch object, robot stop and program exit! \n");
                                                // exit(0);
                                        }
                                }
                                // printf("\n");

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
                        if ( ! foo_act_vel_ref_body) { cout << "文件不能打开" <<endl; }
                        foo_act_vel_ref_body <<  
                        body_root.foot_act_vel(0,1) <<" "<< 
                        body_root.foot_act_vel(1,1) <<" "<< 
                        body_root.foot_act_vel(2,1) << endl;
                        // foo_act_vel_ref_body <<  body_root.foot_act_vel(0,1) <<" "<< body_root.foot_act_vel(1,1) <<" "<< body_root.foot_act_vel(2,1) << endl;
                        
                        // printf("_Lf.f.joi_act_pos[0]:%f  _Lf.f.joi_act_pos[1]:%f _Lf.f.joi_act_pos[2]:%f  \n",
                        //         _Lf.f.joi_act_pos[0]*_RAD2,_Lf.f.joi_act_pos[1]*_RAD2,_Lf.f.joi_act_pos[2]*_RAD2);

                        // if ( ! foo_act_for) { cout << "文件不能打开" <<endl; }
                        // foo_act_for <<  leg_root.foot_act_force(0,1) <<" "<< leg_root.foot_act_force(1,1) <<" "<< leg_root.foot_act_force(2,1) << endl;

                        if ( ! foo_tip_act_pos_ref_leg_ori) { cout << "文件不能打开" <<endl; }
                        foo_tip_act_pos_ref_leg_ori << 
                        leg_root.foot_act_pos(0,1) <<" "<< 
                        leg_root.foot_act_pos(1,1) <<" "<< 
                        leg_root.foot_act_pos(2,1) << endl;
                       
                        // std::cout<<"leg_root.foot_act_pos: "<<std::endl;
                        // std::cout<<leg_root.foot_act_pos(0,2)<<std::endl;
                        // std::cout<<leg_root.foot_act_pos(1,2)<<std::endl;
                        // std::cout<<leg_root.foot_act_pos(2,2)<<std::endl;
                #endif
                printf("--------next-------------\n");
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
                _Cpg.control_cycle=_Cpg.CtrlCyclLinTran.linearConvert(_Cpg.control_cycle,set_cpg_ctrl_cycle,180); //lcc 20230418:设计一个周期点数
                _Cpg.T=_Cpg.TLinTran.linearConvert(_Cpg.T,1,180);
        #endif

        keyBoardControl(KeyBoardCtrl.retKeyValue()); //lcc 20230409:跟据键盘指令，控制机器人

        if(set_para_init_flag==1)   //lcc 初始设置步长和步高的ｋ值
        {
                set_x_deviation=0; set_y_deviation=0; set_z_deviation=0; 
                set_yaw=0; set_roll=0; set_pitch=0;
                set_step_length_k=1.1;
                set_step_hight_k=1.3;
                set_para_init_flag=0;
                #if HARD_WARE==1
                        set_cpg_ctrl_cycle=0.01;
                #elif HARD_WARE==2
                        set_cpg_ctrl_cycle=0.002;   //lcc 一个步态周期的点数＝１／set_cpg_ctrl_cycle
                #endif
        }

        _LfTraj.length_k=_LfStepLKTrans.linearConvert(_LfTraj.length_k,set_step_length_k,60);
        _LmTraj.length_k=_LmStepLKTrans.linearConvert(_LmTraj.length_k,set_step_length_k,60);
        _LbTraj.length_k=_LbStepLKTrans.linearConvert(_LbTraj.length_k,set_step_length_k,60);
        _RfTraj.length_k=_RfStepLKTrans.linearConvert(_RfTraj.length_k,set_step_length_k,60);
        _RmTraj.length_k=_RmStepLKTrans.linearConvert(_RmTraj.length_k,set_step_length_k,60);
        _RbTraj.length_k=_RbStepLKTrans.linearConvert(_RbTraj.length_k,set_step_length_k,60);

        _LfTraj.height_k=_LfStepWKTrans.linearConvert(_LfTraj.height_k,set_step_hight_k,60);
        _LmTraj.height_k=_LmStepWKTrans.linearConvert(_LmTraj.height_k,set_step_hight_k,60);
        _LbTraj.height_k=_LbStepWKTrans.linearConvert(_LbTraj.height_k,set_step_hight_k,60);
        _RfTraj.height_k=_RfStepWKTrans.linearConvert(_RfTraj.height_k,set_step_hight_k,60);
        _RmTraj.height_k=_RmStepWKTrans.linearConvert(_RmTraj.height_k,set_step_hight_k,60);
        _RbTraj.height_k=_RbStepWKTrans.linearConvert(_RbTraj.height_k,set_step_hight_k,60);

        //  lcc 20230330:机器人yaw,roll,pitch姿态的调整
        _Rb._FooBodAdjMap.rrr_yaw=_Rb._FooBodAdjMap.YawLinTran.linearConvert(_Rb._FooBodAdjMap.rrr_yaw,set_yaw,60);
        _Rb._FooBodAdjMap.rrr_roll=_Rb._FooBodAdjMap.RolLinTran.linearConvert(_Rb._FooBodAdjMap.rrr_roll,set_roll,60);
        _Rb._FooBodAdjMap.rrr_pitch=_Rb._FooBodAdjMap.PitLinTran.linearConvert(_Rb._FooBodAdjMap.rrr_pitch,set_pitch,60);
        FooTipAndBodAdjMap::fuselageAttiuCtrl(_Rb._FooBodAdjMap.rrr_yaw,_Rb._FooBodAdjMap.rrr_roll,_Rb._FooBodAdjMap.rrr_pitch);
        // printf("_Rb._FooBodAdjMap.rrr_roll:%f  _Rb._FooBodAdjMap.rrr_pitch:%f \n",_Rb._FooBodAdjMap.rrr_roll,_Rb._FooBodAdjMap.rrr_pitch);

        // lcc 20230330:机器人姿态的平移调整
        _Rb._FooBodAdjMap.x_deviation=_Rb._FooBodAdjMap.XdeLinTran.linearConvert(_Rb._FooBodAdjMap.x_deviation,set_x_deviation,60);
        _Rb._FooBodAdjMap.y_deviation=_Rb._FooBodAdjMap.YdeLinTran.linearConvert(_Rb._FooBodAdjMap.y_deviation,set_y_deviation,60);
        _Rb._FooBodAdjMap.z_deviation=_Rb._FooBodAdjMap.ZdeLinTran.linearConvert(_Rb._FooBodAdjMap.z_deviation,set_z_deviation,60);
        FooTipAndBodAdjMap::fuselageDeviCtrl(_Rb._FooBodAdjMap.x_deviation,_Rb._FooBodAdjMap.y_deviation,_Rb._FooBodAdjMap.z_deviation);
        // printf("_Rb._FooBodAdjMap.z_deviation:%f   \n",_Rb._FooBodAdjMap.z_deviation);
}



