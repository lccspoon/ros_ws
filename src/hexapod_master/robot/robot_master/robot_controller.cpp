#include"robot_controller.h"
// #include"../hardware/hardware_bridge.h"

/**
* @brief 
* @details 基于轨迹得到各电机的期望角度; 
* @details 机器人启动时，会根据当前电机真实角度进入启动姿态; 
* @details 通过sendDataConPro()对joi_des_pos的连续性进行保护; 
* @author lcc
* @date date at : 20230329
*/
Eigen::Matrix<double,3,6>  joi_des_pos_vice;

void Hexapod::trajectoryPlaning(void) 
{
    #if ONLY_Quadruped==1  //机器人不不可切换步态，仅仅可用四足步态
        _Cpg.gait=_Cpg.GaitLinTran.linearConvert(_Cpg.gait,0.666,100); 
        leg_root.foot_swing_traj.block<3,1>(0,0)=neur_bezier[0].bezierCurve(cpg_scheduler(0,4),cpg_scheduler(1,4) )*_FooBodAdjMap[0].step_amplitude;  
        leg_root.foot_swing_traj.block<3,1>(0,1)=neur_bezier[1].bezierCurve(cpg_scheduler(0,3),cpg_scheduler(1,3) )*_FooBodAdjMap[1].step_amplitude;  
        leg_root.foot_swing_traj.block<3,1>(0,2)=neur_bezier[2].bezierCurve(cpg_scheduler(0,5),cpg_scheduler(1,5) )*_FooBodAdjMap[2].step_amplitude;
        leg_root.foot_swing_traj.block<3,1>(0,3)=neur_bezier[3].bezierCurve(cpg_scheduler(0,2),cpg_scheduler(1,2) )*_FooBodAdjMap[3].step_amplitude; 
        leg_root.foot_swing_traj.block<3,1>(0,4)=neur_bezier[4].bezierCurve(cpg_scheduler(0,1),cpg_scheduler(1,1) )*_FooBodAdjMap[4].step_amplitude;
        leg_root.foot_swing_traj.block<3,1>(0,5)=neur_bezier[5].bezierCurve(cpg_scheduler(0,0),cpg_scheduler(1,0) )*_FooBodAdjMap[5].step_amplitude;
    #elif ONLY_Quadruped==0
        leg_root.foot_swing_traj.block<3,1>(0,0)=neur_bezier[0].bezierCurve(cpg_scheduler(0,3),cpg_scheduler(1,3) )*_FooBodAdjMap[0].step_amplitude;  
        leg_root.foot_swing_traj.block<3,1>(0,1)=neur_bezier[1].bezierCurve(cpg_scheduler(0,4),cpg_scheduler(1,4) )*_FooBodAdjMap[1].step_amplitude;  
        leg_root.foot_swing_traj.block<3,1>(0,2)=neur_bezier[2].bezierCurve(cpg_scheduler(0,5),cpg_scheduler(1,5) )*_FooBodAdjMap[2].step_amplitude;
        leg_root.foot_swing_traj.block<3,1>(0,3)=neur_bezier[3].bezierCurve(cpg_scheduler(0,2),cpg_scheduler(1,2) )*_FooBodAdjMap[3].step_amplitude; 
        leg_root.foot_swing_traj.block<3,1>(0,4)=neur_bezier[4].bezierCurve(cpg_scheduler(0,1),cpg_scheduler(1,1) )*_FooBodAdjMap[4].step_amplitude;
        leg_root.foot_swing_traj.block<3,1>(0,5)=neur_bezier[5].bezierCurve(cpg_scheduler(0,0),cpg_scheduler(1,0) )*_FooBodAdjMap[5].step_amplitude;
    #endif

        // #if ADAPTIV_FLAG==1
            // adaptive_control();
        // #else
            leg_root.foot_trajectory.block<3,1>(0,0)=leg_root.foot_swing_traj.block<3,1>(0,0);
            leg_root.foot_trajectory.block<3,1>(0,1)=leg_root.foot_swing_traj.block<3,1>(0,1);
            leg_root.foot_trajectory.block<3,1>(0,2)=leg_root.foot_swing_traj.block<3,1>(0,2);
            leg_root.foot_trajectory.block<3,1>(0,3)=leg_root.foot_swing_traj.block<3,1>(0,3);
            leg_root.foot_trajectory.block<3,1>(0,4)=leg_root.foot_swing_traj.block<3,1>(0,4);
            leg_root.foot_trajectory.block<3,1>(0,5)=leg_root.foot_swing_traj.block<3,1>(0,5); 
        // #endif

        _FooBodAdjMap[0].setInitEndEfforeAndPBias
        (leg_root.foot_static_pos.block<3,1>(0,0),NULL, fuselage_length/2,fuselage_width/2);
        _FooBodAdjMap[1].setInitEndEfforeAndPBias
        (leg_root.foot_static_pos.block<3,1>(0,1),NULL, 0, fuselage_width/2);
        _FooBodAdjMap[2].setInitEndEfforeAndPBias
        (leg_root.foot_static_pos.block<3,1>(0,2),NULL, -fuselage_length/2, fuselage_width/2);
        _FooBodAdjMap[3].setInitEndEfforeAndPBias
        (leg_root.foot_static_pos.block<3,1>(0,3),NULL, fuselage_length/2,-fuselage_width/2);
        _FooBodAdjMap[4].setInitEndEfforeAndPBias
        (leg_root.foot_static_pos.block<3,1>(0,4),NULL, 0,-fuselage_width/2);
        _FooBodAdjMap[5].setInitEndEfforeAndPBias
        (leg_root.foot_static_pos.block<3,1>(0,5),NULL, -fuselage_length/2,-fuselage_width/2);

        leg_root.foot_traj_mapping_to_body.block<3,1>(0,5)=_FooBodAdjMap[5].trajAndAdjustMapping(leg_root.foot_trajectory.block<3,1>(0,5));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,4)=_FooBodAdjMap[4].trajAndAdjustMapping(leg_root.foot_trajectory.block<3,1>(0,4));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,3)=_FooBodAdjMap[3].trajAndAdjustMapping(leg_root.foot_trajectory.block<3,1>(0,3));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,2)=_FooBodAdjMap[2].trajAndAdjustMapping(leg_root.foot_trajectory.block<3,1>(0,2));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,1)=_FooBodAdjMap[1].trajAndAdjustMapping(leg_root.foot_trajectory.block<3,1>(0,1));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,0)=_FooBodAdjMap[0].trajAndAdjustMapping(leg_root.foot_trajectory.block<3,1>(0,0));
        // std::cout<<"leg_root.foot_traj_mapping_to_body: "<<std::endl;
        // std::cout<<leg_root.foot_traj_mapping_to_body<<std::endl;

        joi_des_pos_vice.block<3,1>(0,5)=_kinematic[5].invKinematic(5, leg_root.foot_traj_mapping_to_body.block<3,1>(0,5));
        joi_des_pos_vice.block<3,1>(0,4)=_kinematic[4].invKinematic(4, leg_root.foot_traj_mapping_to_body.block<3,1>(0,4));
        joi_des_pos_vice.block<3,1>(0,3)=_kinematic[3].invKinematic(3, leg_root.foot_traj_mapping_to_body.block<3,1>(0,3));
        joi_des_pos_vice.block<3,1>(0,2)=_kinematic[2].invKinematic(2, leg_root.foot_traj_mapping_to_body.block<3,1>(0,2));
        joi_des_pos_vice.block<3,1>(0,1)=_kinematic[1].invKinematic(1, leg_root.foot_traj_mapping_to_body.block<3,1>(0,1));
        joi_des_pos_vice.block<3,1>(0,0)=_kinematic[0].invKinematic(0, leg_root.foot_traj_mapping_to_body.block<3,1>(0,0));

    //lcc 20230405:      以下程序让机器人能在启动时，读去当前角度并且从当前角度缓慢启动到站立姿态
    #if HARD_WARE==1
            joi_des_pos_vice.block<3,1>(0,0)=-joi_des_pos_vice.block<3,1>(0,0)+HarBri.lf_off_set;
            joi_des_pos_vice.block<3,1>(0,1)=-joi_des_pos_vice.block<3,1>(0,1)+HarBri.lm_off_set;
            joi_des_pos_vice.block<3,1>(0,2)=-joi_des_pos_vice.block<3,1>(0,2)+HarBri.lb_off_set;
            joi_des_pos_vice.block<3,1>(0,3)=joi_des_pos_vice.block<3,1>(0,3)+HarBri.rf_off_set;
            joi_des_pos_vice.block<3,1>(0,4)=joi_des_pos_vice.block<3,1>(0,4)+HarBri.rm_off_set;
            joi_des_pos_vice.block<3,1>(0,5)=joi_des_pos_vice.block<3,1>(0,5)+HarBri.rb_off_set;
            // joi_des_pos_vice.block<3,1>(0,0)=-joi_des_pos_vice.block<3,1>(0,0);
            // joi_des_pos_vice.block<3,1>(0,1)=-joi_des_pos_vice.block<3,1>(0,1);
            // joi_des_pos_vice.block<3,1>(0,2)=-joi_des_pos_vice.block<3,1>(0,2);
            // joi_des_pos_vice.block<3,1>(0,3)=joi_des_pos_vice.block<3,1>(0,3);
            // joi_des_pos_vice.block<3,1>(0,4)=joi_des_pos_vice.block<3,1>(0,4);
            // joi_des_pos_vice.block<3,1>(0,5)=joi_des_pos_vice.block<3,1>(0,5);
    #elif HARD_WARE==2

    #endif
    motor_pos_set_count=1200;
    if(get_joint_pos_flag==true && joint_pos_run_count!=motor_pos_set_count)
    {
        leg_root.joint_des_pos.block<3,1>(0,0)=_start_pos_conver[0].linearConvert(leg_root.joint_des_pos.block<3,1>(0,0), joi_des_pos_vice.block<3,1>(0,0), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,1)=_start_pos_conver[1].linearConvert(leg_root.joint_des_pos.block<3,1>(0,1), joi_des_pos_vice.block<3,1>(0,1), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,2)=_start_pos_conver[2].linearConvert(leg_root.joint_des_pos.block<3,1>(0,2), joi_des_pos_vice.block<3,1>(0,2), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,3)=_start_pos_conver[3].linearConvert(leg_root.joint_des_pos.block<3,1>(0,3), joi_des_pos_vice.block<3,1>(0,3), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,4)=_start_pos_conver[4].linearConvert(leg_root.joint_des_pos.block<3,1>(0,4), joi_des_pos_vice.block<3,1>(0,4), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,5)=_start_pos_conver[5].linearConvert(leg_root.joint_des_pos.block<3,1>(0,5), joi_des_pos_vice.block<3,1>(0,5), motor_pos_set_count);
        joint_pos_run_count++;
    }
    else if(joint_pos_run_count==motor_pos_set_count)
    {   
        motor_pos_ach_count_flag=1;
        leg_root.joint_des_pos=joi_des_pos_vice;
    }

    //lcc 20230405:如接motor_pos_get_flag==true 意味着所有电机都接收到了消息，那么对joi_des_pos的连续性进行保护
    if(get_joint_pos_flag==true) 
    {
        leg_root.joint_des_pos.block<3,1>(0,5)=_dataUnuProtect[5].sendDataConPro(5,leg_root.joint_des_pos.block<3,1>(0,5),10*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,4)=_dataUnuProtect[4].sendDataConPro(4,leg_root.joint_des_pos.block<3,1>(0,4),10*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,3)=_dataUnuProtect[3].sendDataConPro(3,leg_root.joint_des_pos.block<3,1>(0,3),10*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,2)=_dataUnuProtect[2].sendDataConPro(2,leg_root.joint_des_pos.block<3,1>(0,2),10*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,1)=_dataUnuProtect[1].sendDataConPro(1,leg_root.joint_des_pos.block<3,1>(0,1),10*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,0)=_dataUnuProtect[0].sendDataConPro(0,leg_root.joint_des_pos.block<3,1>(0,0),10*_RAD1);
    }

    //lcc 20230405: 对joi_des_pos的连续性进行保护时，但凡有一个电机的joi_des_pos的相邻两次角度差值过大，所有电机都会stop
    if(_dataUnuProtect[5].StopFlag==false or _dataUnuProtect[4].StopFlag==false or _dataUnuProtect[3].StopFlag==false
    or _dataUnuProtect[2].StopFlag==false or _dataUnuProtect[1].StopFlag==false or _dataUnuProtect[0].StopFlag==false)
    {
        _dataUnuProtect[5].StopFlag=false;_dataUnuProtect[4].StopFlag=false;_dataUnuProtect[3].StopFlag=false;
        _dataUnuProtect[2].StopFlag=false;_dataUnuProtect[1].StopFlag=false;_dataUnuProtect[0].StopFlag=false;
    }
}

void Hexapod::positionController(void)
{
        leg_root.joint_msg_pub=leg_root.joint_des_pos;
}

void Hexapod::adaController(void)//zyt_adacontrol 2023.4.19
{
    #if STRUCT==1

    double ada_a=0.05,ada_b=15,ada_beta=0.05; 
    double kp1=80,kp2=80,kp3=80;
    double lkd1=10,lkd2=10,lkd3=10;
    double rkd1=-10,rkd2=-10,rkd3=-10;
    _LfAdaCtrl.setAda(ada_beta,ada_a,ada_b);_LfAdaCtrl.setKp(kp1,kp2,kp3);_LfAdaCtrl.setKd(lkd1,lkd2,lkd3);
    _LmAdadCtrl.setAda(ada_beta,ada_a,ada_b);_LmAdadCtrl.setKp(kp1,kp2,kp3);_LmAdadCtrl.setKd(lkd1,lkd2,lkd3);
    _LbAdaCtrl.setAda(ada_beta,ada_a,ada_b);_LbAdaCtrl.setKp(kp1,kp2,kp3);_LbAdaCtrl.setKd(lkd1,lkd2,lkd3);

    _RfAdaCtrl.setAda(-ada_beta,ada_a,ada_b);_RfAdaCtrl.setKp(kp1,kp2,kp3);_RfAdaCtrl.setKd(rkd1,rkd2,rkd3);
    _RmAdaCtrl.setAda(-ada_beta,ada_a,ada_b);_RmAdaCtrl.setKp(kp1,kp2,kp3);_RmAdaCtrl.setKd(rkd1,rkd2,rkd3);
    _RbAdaCtrl.setAda(-ada_beta,ada_a,ada_b);_RbAdaCtrl.setKp(kp1,kp2,kp3);_RbAdaCtrl.setKd(rkd1,rkd2,rkd3);

    _Lf.p.joi_des_tor=
                _LfAdaCtrl.adaTau(_Lf.f.joi_act_pos, _Lf.f.joi_des_pos, _Lf.f.joi_act_vel, NULL);
    _Lm.p.joi_des_tor=
                _LmAdadCtrl.adaTau(_Lm.f.joi_act_pos, _Lm.f.joi_des_pos, _Lm.f.joi_act_vel, NULL);
    _Lb.p.joi_des_tor=
                _LbAdaCtrl.adaTau(_Lb.f.joi_act_pos, _Lb.f.joi_des_pos, _Lb.f.joi_act_vel, NULL);
    _Rf.p.joi_des_tor=
                _RfAdaCtrl.adaTau(_Rf.f.joi_act_pos, _Rf.f.joi_des_pos, _Rf.f.joi_act_vel, NULL);
    _Rm.p.joi_des_tor=
                _RmAdaCtrl.adaTau(_Rm.f.joi_act_pos, _Rm.f.joi_des_pos, _Rm.f.joi_act_vel, NULL);
    _Rb.p.joi_des_tor=
                _RbAdaCtrl.adaTau(_Rb.f.joi_act_pos, _Rb.f.joi_des_pos, _Rb.f.joi_act_vel, NULL);

    for(int i=0;i<3;i++) _Rb.f.joi_des_tor[i]=_Rb.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rm.f.joi_des_tor[i]=_Rm.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rf.f.joi_des_tor[i]=_Rf.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lb.f.joi_des_tor[i]=_Lb.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lm.f.joi_des_tor[i]=_Lm.p.joi_des_tor[i];  
    for(int i=0;i<3;i++) _Lf.f.joi_des_tor[i]=_Lf.p.joi_des_tor[i];

    for(int i=0;i<3;i++) _Rb.f.joi_msg_pub[i]=_Rb.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rm.f.joi_msg_pub[i]=_Rm.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rf.f.joi_msg_pub[i]=_Rf.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lb.f.joi_msg_pub[i]=_Lb.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lm.f.joi_msg_pub[i]=_Lm.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lf.f.joi_msg_pub[i]=_Lf.f.joi_des_tor[i]; 
    // cout<<"toque:"<<_Rm.f.joi_des_tor[1]<<endl;
    #elif STRUCT==2
    double ada_a=0.05,ada_b=15,ada_beta=0.05; 
    double kp1=80,kp2=80,kp3=80;
    double lkd1=10,lkd2=10,lkd3=10;
    double rkd1=-10,rkd2=-10,rkd3=-10;
    _LfAdaCtrl.setAda(ada_beta,ada_a,ada_b);_LfAdaCtrl.setKp(kp1,kp2,kp3);_LfAdaCtrl.setKd(lkd1,lkd2,lkd3);
    _LmAdadCtrl.setAda(ada_beta,ada_a,ada_b);_LmAdadCtrl.setKp(kp1,kp2,kp3);_LmAdadCtrl.setKd(lkd1,lkd2,lkd3);
    _LbAdaCtrl.setAda(ada_beta,ada_a,ada_b);_LbAdaCtrl.setKp(kp1,kp2,kp3);_LbAdaCtrl.setKd(lkd1,lkd2,lkd3);

    _RfAdaCtrl.setAda(-ada_beta,ada_a,ada_b);_RfAdaCtrl.setKp(kp1,kp2,kp3);_RfAdaCtrl.setKd(rkd1,rkd2,rkd3);
    _RmAdaCtrl.setAda(-ada_beta,ada_a,ada_b);_RmAdaCtrl.setKp(kp1,kp2,kp3);_RmAdaCtrl.setKd(rkd1,rkd2,rkd3);
    _RbAdaCtrl.setAda(-ada_beta,ada_a,ada_b);_RbAdaCtrl.setKp(kp1,kp2,kp3);_RbAdaCtrl.setKd(rkd1,rkd2,rkd3);

    _Lf.p.joi_des_tor=
                _LfAdaCtrl.adaTau(_Lf.f.joi_act_pos, _Lf.f.joi_des_pos, _Lf.f.joi_act_vel, NULL);
    _Lm.p.joi_des_tor=
                _LmAdadCtrl.adaTau(_Lm.f.joi_act_pos, _Lm.f.joi_des_pos, _Lm.f.joi_act_vel, NULL);
    _Lb.p.joi_des_tor=
                _LbAdaCtrl.adaTau(_Lb.f.joi_act_pos, _Lb.f.joi_des_pos, _Lb.f.joi_act_vel, NULL);
    _Rf.p.joi_des_tor=
                _RfAdaCtrl.adaTau(_Rf.f.joi_act_pos, _Rf.f.joi_des_pos, _Rf.f.joi_act_vel, NULL);
    _Rm.p.joi_des_tor=
                _RmAdaCtrl.adaTau(_Rm.f.joi_act_pos, _Rm.f.joi_des_pos, _Rm.f.joi_act_vel, NULL);
    _Rb.p.joi_des_tor=
                _RbAdaCtrl.adaTau(_Rb.f.joi_act_pos, _Rb.f.joi_des_pos, _Rb.f.joi_act_vel, NULL);

    for(int i=0;i<3;i++) _Rb.f.joi_des_tor[i]=_Rb.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rm.f.joi_des_tor[i]=_Rm.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rf.f.joi_des_tor[i]=_Rf.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lb.f.joi_des_tor[i]=_Lb.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lm.f.joi_des_tor[i]=_Lm.p.joi_des_tor[i];  
    for(int i=0;i<3;i++) _Lf.f.joi_des_tor[i]=_Lf.p.joi_des_tor[i];

    for(int i=0;i<3;i++) _Rb.f.joi_msg_pub[i]=_Rb.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rm.f.joi_msg_pub[i]=_Rm.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rf.f.joi_msg_pub[i]=_Rf.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lb.f.joi_msg_pub[i]=_Lb.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lm.f.joi_msg_pub[i]=_Lm.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lf.f.joi_msg_pub[i]=_Lf.f.joi_des_tor[i]; 
    // cout<<"toque:"<<_Rm.f.joi_des_tor[1]<<endl;
    #endif
}

void Hexapod::pdController(void)
{
    #if STRUCT==1
    // _LfPdCtrl.setKp(2,4,2);_LfPdCtrl.setKd(0.2,0.4,0.2);_LfPdCtrl.setKi(0,0,0);
    // _LmPdCtrl.setKp(2,4,2);_LmPdCtrl.setKd(0.2,0.4,0.2);_LmPdCtrl.setKi(0,0,0);
    // _LbPdCtrl.setKp(2,4,2);_LbPdCtrl.setKd(0.2,0.4,0.2);_LbPdCtrl.setKi(0,0,0);

    // _RfPdCtrl.setKp(2,4,2);_RfPdCtrl.setKd(-0.2,-0.4,-0.2);_RfPdCtrl.setKi(0,0,0);
    // _RmPdCtrl.setKp(2,4,2);_RmPdCtrl.setKd(-0.2,-0.4,-0.2);_RmPdCtrl.setKi(0,0,0);
    // _RbPdCtrl.setKp(2,4,2);_RbPdCtrl.setKd(-0.2,-0.4,-0.2);_RbPdCtrl.setKi(0,0,0);

    // _Lf.p.joi_des_tor=
    //             _LfPdCtrl.pdTau(_Lf.f.joi_act_pos, _Lf.f.joi_des_pos, _Lf.f.joi_act_vel, NULL);
    // _Lm.p.joi_des_tor=
    //             _LmPdCtrl.pdTau(_Lm.f.joi_act_pos, _Lm.f.joi_des_pos, _Lm.f.joi_act_vel, NULL);
    // _Lb.p.joi_des_tor=
    //             _LbPdCtrl.pdTau(_Lb.f.joi_act_pos, _Lb.f.joi_des_pos, _Lb.f.joi_act_vel, NULL);
    // _Rf.p.joi_des_tor=
    //             _RfPdCtrl.pdTau(_Rf.f.joi_act_pos, _Rf.f.joi_des_pos, _Rf.f.joi_act_vel, NULL);
    // _Rm.p.joi_des_tor=
    //             _RmPdCtrl.pdTau(_Rm.f.joi_act_pos, _Rm.f.joi_des_pos, _Rm.f.joi_act_vel, NULL);
    // _Rb.p.joi_des_tor=
    //             _RbPdCtrl.pdTau(_Rb.f.joi_act_pos, _Rb.f.joi_des_pos, _Rb.f.joi_act_vel, NULL);


    _LfPdCtrl.setKp(70,70,70);_LfPdCtrl.setKd(500,500,500);_LfPdCtrl.setKi(0.03,0.03,0.03);
    _LmPdCtrl.setKp(70,70,70);_LmPdCtrl.setKd(500,500,500);_LmPdCtrl.setKi(0.03,0.03,0.03);
    _LbPdCtrl.setKp(70,70,70);_LbPdCtrl.setKd(500,500,500);_LbPdCtrl.setKi(0.03,0.03,0.03);

    _RfPdCtrl.setKp(70,70,70);_RfPdCtrl.setKd(500,500,500);_RfPdCtrl.setKi(0.03,0.03,0.03);
    _RmPdCtrl.setKp(70,70,70);_RmPdCtrl.setKd(500,500,500);_RmPdCtrl.setKi(0.03,0.03,0.03);
    _RbPdCtrl.setKp(70,70,70);_RbPdCtrl.setKd(500,500,500);_RbPdCtrl.setKi(0.03,0.03,0.03);

    _Lf.p.joi_des_tor=
                _LfPdCtrl.positonPidTau(_Lf.f.joi_act_pos, _Lf.f.joi_des_pos, _Lf.f.joi_act_vel, NULL);
    _Lm.p.joi_des_tor=
                _LmPdCtrl.positonPidTau(_Lm.f.joi_act_pos, _Lm.f.joi_des_pos, _Lm.f.joi_act_vel, NULL);
    _Lb.p.joi_des_tor=
                _LbPdCtrl.positonPidTau(_Lb.f.joi_act_pos, _Lb.f.joi_des_pos, _Lb.f.joi_act_vel, NULL);
    _Rf.p.joi_des_tor=
                _RfPdCtrl.positonPidTau(_Rf.f.joi_act_pos, _Rf.f.joi_des_pos, _Rf.f.joi_act_vel, NULL);
    _Rm.p.joi_des_tor=
                _RmPdCtrl.positonPidTau(_Rm.f.joi_act_pos, _Rm.f.joi_des_pos, _Rm.f.joi_act_vel, NULL);
    _Rb.p.joi_des_tor=
                _RbPdCtrl.positonPidTau(_Rb.f.joi_act_pos, _Rb.f.joi_des_pos, _Rb.f.joi_act_vel, NULL);    


    for(int i=0;i<3;i++) _Rb.f.joi_des_tor[i]=_Rb.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rm.f.joi_des_tor[i]=_Rm.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rf.f.joi_des_tor[i]=_Rf.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lb.f.joi_des_tor[i]=_Lb.p.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lm.f.joi_des_tor[i]=_Lm.p.joi_des_tor[i];  
    for(int i=0;i<3;i++) _Lf.f.joi_des_tor[i]=_Lf.p.joi_des_tor[i];

    for(int i=0;i<3;i++) _Rb.f.joi_msg_pub[i]=_Rb.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rm.f.joi_msg_pub[i]=_Rm.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Rf.f.joi_msg_pub[i]=_Rf.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lb.f.joi_msg_pub[i]=_Lb.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lm.f.joi_msg_pub[i]=_Lm.f.joi_des_tor[i];
    for(int i=0;i<3;i++) _Lf.f.joi_msg_pub[i]=_Lf.f.joi_des_tor[i]; 
    #elif STRUCT==3

    // _LfPdCtrl.setKp(0,10,0);_LfPdCtrl.setKd(0,1,0);_LfPdCtrl.setKi(0.00,0.00,0.00);

    // _LmPdCtrl.setKp(00,00,00);_LmPdCtrl.setKd(000,000,000);_LmPdCtrl.setKi(0.00,0.00,0.00);
    // _LbPdCtrl.setKp(00,00,00);_LbPdCtrl.setKd(000,000,000);_LbPdCtrl.setKi(0.00,0.00,0.00);

    // _RfPdCtrl.setKp(00,00,00);_RfPdCtrl.setKd(000,000,000);_RfPdCtrl.setKi(0.00,0.00,0.00);
    // _RmPdCtrl.setKp(00,00,00);_RmPdCtrl.setKd(000,000,000);_RmPdCtrl.setKi(0.00,0.00,0.00);
    // _RbPdCtrl.setKp(00,00,00);_RbPdCtrl.setKd(000,000,000);_RbPdCtrl.setKi(0.00,0.00,0.00);

    // _Lf.p.joi_des_tor=
    //             _LfPdCtrl.positonPidTau(_Lf.f.joi_act_pos, _Lf.f.joi_des_pos, _Lf.f.joi_act_vel, NULL);
    // _Lm.p.joi_des_tor=
    //             _LmPdCtrl.positonPidTau(_Lm.f.joi_act_pos, _Lm.f.joi_des_pos, _Lm.f.joi_act_vel, NULL);
    // _Lb.p.joi_des_tor=
    //             _LbPdCtrl.positonPidTau(_Lb.f.joi_act_pos, _Lb.f.joi_des_pos, _Lb.f.joi_act_vel, NULL);
    // _Rf.p.joi_des_tor=
    //             _RfPdCtrl.positonPidTau(_Rf.f.joi_act_pos, _Rf.f.joi_des_pos, _Rf.f.joi_act_vel, NULL);
    // _Rm.p.joi_des_tor=
    //             _RmPdCtrl.positonPidTau(_Rm.f.joi_act_pos, _Rm.f.joi_des_pos, _Rm.f.joi_act_vel, NULL);
    // _Rb.p.joi_des_tor=
    //             _RbPdCtrl.positonPidTau(_Rb.f.joi_act_pos, _Rb.f.joi_des_pos, _Rb.f.joi_act_vel, NULL);    


    // for(int i=0;i<3;i++) _Rb.f.joi_des_tor[i]=_Rb.p.joi_des_tor[i];
    // for(int i=0;i<3;i++) _Rm.f.joi_des_tor[i]=_Rm.p.joi_des_tor[i];
    // for(int i=0;i<3;i++) _Rf.f.joi_des_tor[i]=_Rf.p.joi_des_tor[i];
    // for(int i=0;i<3;i++) _Lb.f.joi_des_tor[i]=_Lb.p.joi_des_tor[i];
    // for(int i=0;i<3;i++) _Lm.f.joi_des_tor[i]=_Lm.p.joi_des_tor[i];  
    // for(int i=0;i<3;i++) _Lf.f.joi_des_tor[i]=_Lf.p.joi_des_tor[i];

    // for(int i=0;i<3;i++) _Rb.f.joi_msg_pub[i]=_Rb.f.joi_des_tor[i];
    // for(int i=0;i<3;i++) _Rm.f.joi_msg_pub[i]=_Rm.f.joi_des_tor[i];
    // for(int i=0;i<3;i++) _Rf.f.joi_msg_pub[i]=_Rf.f.joi_des_tor[i];
    // for(int i=0;i<3;i++) _Lb.f.joi_msg_pub[i]=_Lb.f.joi_des_tor[i];
    // for(int i=0;i<3;i++) _Lm.f.joi_msg_pub[i]=_Lm.f.joi_des_tor[i];
    // for(int i=0;i<3;i++) _Lf.f.joi_msg_pub[i]=_Lf.f.joi_des_tor[i]; 

    #endif


}


/**
* @brief 
* @details 姿态调整，调整足端位置来调整姿态
* @author lcc
* @date date at : 20230625
*/
double pitch_temp=0;
double adp_pitch=0;
double adp_pitch_vice=0;
double adp_roll=0;
double adp_yaw=0;
linear_trans _adp_pitch_conver;
linear_trans pitch_kpkikd_conver[3];
double pitch_kp=0,pitch_ki=0,pitch_kd=0;
linear_trans roll_kpkikd_conver[3];
double roll_kp=0,roll_ki=0,roll_kd=0;
void Hexapod::attitudeAdjust(void)
{
    #if HARD_WARE==1
            // 姿态控制,pid实现闭环姿态控制.
            // -- 以下是x轴转角的调整 --//
            double des_roll_temp; 
            des_roll_temp=body_root.eulerAngle(0);
            if(des_roll_temp*_RAD2>=10) des_roll_temp=10*_RAD1;
            else if(des_roll_temp*_RAD2<=-10) des_roll_temp=-10*_RAD1;

            roll_kp = roll_kpkikd_conver[0].linearConvert(roll_kp,1,1000  );
            roll_ki = roll_kpkikd_conver[1].linearConvert(roll_ki,0.3,1000  );
            roll_kd = roll_kpkikd_conver[2].linearConvert(roll_kd,0.4,1000  );

            attitude_pid_ctrl[2].setKpKiKd(roll_kp, roll_ki, roll_kd);

            adp_roll=attitude_pid_ctrl[2].incrementPidTau( set_roll,des_roll_temp, dt_s );

            if(adp_roll*_RAD2>= set_roll*_RAD2+5 ) adp_roll= ( set_roll*_RAD2+5 ) *_RAD1;
            else if(adp_roll*_RAD2<= set_roll*_RAD2-5 ) adp_roll= ( set_roll*_RAD2-5 ) *_RAD1;

            // -- 以下是y轴转角的调整 --//
            double des_pitch_temp;
            des_pitch_temp=body_root.eulerAngle(1);
            if(des_pitch_temp*_RAD2>=10) des_pitch_temp=10*_RAD1;
            else if(des_pitch_temp*_RAD2<=-10) des_pitch_temp=-10*_RAD1;

            pitch_kp = pitch_kpkikd_conver[0].linearConvert(pitch_kp,0.8,1000  );
            pitch_ki = pitch_kpkikd_conver[1].linearConvert(pitch_ki,0.2,1000  );
            pitch_kd = pitch_kpkikd_conver[2].linearConvert(pitch_kd,0.3,1000  );

            attitude_pid_ctrl[1].setKpKiKd(pitch_kp, pitch_ki, pitch_kd);

            adp_pitch=attitude_pid_ctrl[1].incrementPidTau( set_pitch, des_pitch_temp , dt_s );

            if(adp_pitch*_RAD2>= set_pitch*_RAD2+5 ) adp_pitch= ( set_pitch*_RAD2+5 ) *_RAD1;
            else if(adp_pitch*_RAD2<= set_pitch*_RAD2-5) adp_pitch= ( set_pitch*_RAD2-5 ) *_RAD1;

            // if( fabs( set_pitch*_RAD2-des_pitch_temp*_RAD2 )>1 )
            {
                    for(int i=0; i<6; i++)
                    {
                            _FooBodAdjMap[i].rrr_yaw=_FooBodAdjMap[i].YawLinTran.linearConvert(_FooBodAdjMap[i].rrr_yaw,set_yaw,1);
                            _FooBodAdjMap[i].rrr_roll=_FooBodAdjMap[i].RolLinTran.linearConvert(_FooBodAdjMap[i].rrr_roll,adp_roll,1);
                            _FooBodAdjMap[i].rrr_pitch=_FooBodAdjMap[i].PitLinTran.linearConvert(_FooBodAdjMap[i].rrr_pitch,adp_pitch,1);
                            _FooBodAdjMap[i].fuselageAttiuCtrl(_FooBodAdjMap[i].rrr_yaw,_FooBodAdjMap[i].rrr_roll,_FooBodAdjMap[i].rrr_pitch);

                            // _FooBodAdjMap[i].x_deviation=_FooBodAdjMap[i].XdeLinTran.linearConvert(_FooBodAdjMap[i].x_deviation,set_x_deviation,1);
                            // _FooBodAdjMap[i].y_deviation=_FooBodAdjMap[i].YdeLinTran.linearConvert(_FooBodAdjMap[i].y_deviation,set_y_deviation,1);
                            // _FooBodAdjMap[i].z_deviation=_FooBodAdjMap[i].ZdeLinTran.linearConvert(_FooBodAdjMap[i].z_deviation,set_z_deviation,1);
                            // // _FooBodAdjMap[i].fuselageDeviCtrl(set_x_deviation,set_y_deviation,set_z_deviation);    
                    }
            }

    #elif HARD_WARE==2
            //lcc 20230518: 在4.4 四元数转欧拉角(Z-Y-X，即RPY)处 存在bug，eulerAngle(2)在正负抖动，使得仿真中的姿态无法控制，待解决
            // -- 以下是x轴转角的调整 --//
            
            // double des_roll_temp; 
            // des_roll_temp=body_root.eulerAngle(2);
            // if(des_roll_temp*_RAD2>=10) des_roll_temp=10*_RAD1;
            // else if(des_roll_temp*_RAD2<=-10) des_roll_temp=-10*_RAD1;

            // attitude_pid_ctrl[2].setKpKiKd(4, 1, 1);
            // // attitude_pid_ctrl[2].setKpKiKd(1, 0.5, 0.5);
            // adp_roll=attitude_pid_ctrl[2].incrementPidTau( set_roll,des_roll_temp, dt_s );

            // if(adp_roll*_RAD2>=15) adp_roll=15*_RAD1;
            // else if(adp_roll*_RAD2<=-15) adp_roll=-15*_RAD1;

            // -- 以下是y轴转角的调整 --//
            double des_pitch_temp;
            des_pitch_temp=body_root.eulerAngle(1);
            if(des_pitch_temp*_RAD2>=10) des_pitch_temp=10*_RAD1;
            else if(des_pitch_temp*_RAD2<=-10) des_pitch_temp=-10*_RAD1;

            attitude_pid_ctrl[1].setKpKiKd(3, 1, 1);
            adp_pitch=attitude_pid_ctrl[1].incrementPidTau( set_pitch, des_pitch_temp , dt_s );

            if(adp_pitch*_RAD2>=10) adp_pitch=10*_RAD1;
            else if(adp_pitch*_RAD2<=-10) adp_pitch=-10*_RAD1;

            for(int i=0; i<6; i++)
            {
                    _FooBodAdjMap[i].rrr_yaw=_FooBodAdjMap[i].YawLinTran.linearConvert(_FooBodAdjMap[i].rrr_yaw,set_yaw,1);
                    _FooBodAdjMap[i].rrr_roll=_FooBodAdjMap[i].RolLinTran.linearConvert(_FooBodAdjMap[i].rrr_roll,adp_roll,1);
                    _FooBodAdjMap[i].rrr_pitch=_FooBodAdjMap[i].PitLinTran.linearConvert(_FooBodAdjMap[i].rrr_pitch,adp_pitch,1);
                    _FooBodAdjMap[i].fuselageAttiuCtrl(_FooBodAdjMap[i].rrr_yaw,_FooBodAdjMap[i].rrr_roll,_FooBodAdjMap[i].rrr_pitch);

                    _FooBodAdjMap[i].x_deviation=_FooBodAdjMap[i].XdeLinTran.linearConvert(_FooBodAdjMap[i].x_deviation,set_x_deviation,1);
                    _FooBodAdjMap[i].y_deviation=_FooBodAdjMap[i].YdeLinTran.linearConvert(_FooBodAdjMap[i].y_deviation,set_y_deviation,1);
                    _FooBodAdjMap[i].z_deviation=_FooBodAdjMap[i].ZdeLinTran.linearConvert(_FooBodAdjMap[i].z_deviation,set_z_deviation,1);
                    _FooBodAdjMap[i].fuselageDeviCtrl(set_x_deviation,set_y_deviation,set_z_deviation);    
            }
            // FooTipAndBodAdjMap::fuselageAttiuCtrl(set_yaw, set_roll, set_pitch);
            // FooTipAndBodAdjMap::fuselageDeviCtrl(set_x_deviation, set_y_deviation, set_z_deviation);
    #endif
}

/**
* @brief 
* @details 摆动腿和支撑腿用不同的kpkd
* @author lcc
* @date date at : 20230625
*/
Eigen::Matrix<double,1,6> cpg_scheduler_last;
Eigen::Matrix<double,1,6> kpkd_adjust_qiqiqi_flag;
Eigen::Matrix<double,1,6> kpkd_adjust_luoluo_flag;
linear_trans joint_kp_conver[6], joint_kd_conver[6];
void Hexapod::kpkdSwitch(void)
{
    int tt=int(1/set_cpg_ctrl_cycle);
    leg_root.joint_supor_kp<<280,280,280;
    leg_root.joint_supor_kd<<3,3,3;
    leg_root.joint_swing_kp<<80,80,80;
    leg_root.joint_swing_kd<<2 ,2 ,2;

    // std::cout<<"leg_real_cpg_signal"<<std::endl;
    // std::cout<<leg_real_cpg_signal<<std::endl;
    // std::cout<<"cpg_touch_down_scheduler"<<std::endl;
    // std::cout<<cpg_touch_down_scheduler<<std::endl;

    for(int i=0;i<6;i++)
    {
            if( cpg_touch_down_scheduler(i)==1 && cpg_scheduler_last(i)==0 )
            {
                    kpkd_adjust_qiqiqi_flag(i)=3;
                    kpkd_adjust_luoluo_flag(i)=0;
            }

            if( cpg_touch_down_scheduler(i)==1 && leg_real_cpg_signal(i)>=-0.9 && kpkd_adjust_qiqiqi_flag(i)==3 )  //如果进入摆动态
            {
                    leg_root.joint_kp.block<3,1>(0,i)=joint_kp_conver[i].linearConvert(leg_root.joint_kp.block<3,1>(0,i),leg_root.joint_swing_kp, tt*0.5*0.5*0.2);
                    leg_root.joint_kd.block<3,1>(0,i)=joint_kd_conver[i].linearConvert(leg_root.joint_kd.block<3,1>(0,i),leg_root.joint_swing_kd, tt*0.5*0.5*0.2);
                    printf("qiqiqiqiqiqiqi\n");
            }

            if( cpg_touch_down_scheduler(i)==1 && leg_real_cpg_signal(i)>=0.0 && leg_real_cpg_signal(i)<=0.1 )
            {
                    kpkd_adjust_qiqiqi_flag(i)=0;
                    kpkd_adjust_luoluo_flag(i)=3;
            }

            if( cpg_touch_down_scheduler(i)==1 && leg_real_cpg_signal(i)>=0.7  && kpkd_adjust_luoluo_flag(i)==3 )
            {
                    leg_root.joint_kp.block<3,1>(0,i)=joint_kp_conver[i].linearConvert(leg_root.joint_kp.block<3,1>(0,i),leg_root.joint_supor_kp, tt*0.5*0.5*0.2);
                    leg_root.joint_kd.block<3,1>(0,i)=joint_kd_conver[i].linearConvert(leg_root.joint_kd.block<3,1>(0,i),leg_root.joint_supor_kd, tt*0.5*0.5*0.2);      
                    printf("luoluoluoluo\n");
            }
            cpg_scheduler_last(i) = cpg_touch_down_scheduler(i);
    }

    std::cout<<"leg_root.joint_kp"<<std::endl;
    std::cout<<leg_root.joint_kp<<std::endl;

    // std::cout<<"kpkd_adjust_qiqiqi_flag"<<std::endl;
    // std::cout<<kpkd_adjust_qiqiqi_flag<<std::endl;
    // std::cout<<"kpkd_adjust_luoluo_flag"<<std::endl;
    // std::cout<<kpkd_adjust_luoluo_flag<<std::endl;
}