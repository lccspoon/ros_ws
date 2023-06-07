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
            adaptive_control();
        // #else
            // leg_root.foot_trajectory.block<3,1>(0,0)=leg_root.foot_swing_traj.block<3,1>(0,0);
            // leg_root.foot_trajectory.block<3,1>(0,1)=leg_root.foot_swing_traj.block<3,1>(0,1);
            // leg_root.foot_trajectory.block<3,1>(0,2)=leg_root.foot_swing_traj.block<3,1>(0,2);
            // leg_root.foot_trajectory.block<3,1>(0,3)=leg_root.foot_swing_traj.block<3,1>(0,3);
            // leg_root.foot_trajectory.block<3,1>(0,4)=leg_root.foot_swing_traj.block<3,1>(0,4);
            // leg_root.foot_trajectory.block<3,1>(0,5)=leg_root.foot_swing_traj.block<3,1>(0,5); 
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
        leg_root.foot_traj_mapping_to_body(1,3)=-leg_root.foot_traj_mapping_to_body(1,3);
        leg_root.foot_traj_mapping_to_body(1,4)=-leg_root.foot_traj_mapping_to_body(1,4);
        leg_root.foot_traj_mapping_to_body(1,5)=-leg_root.foot_traj_mapping_to_body(1,5);
        // std::cout<<"leg_root.foot_traj_mapping_to_body: "<<std::endl;
        // std::cout<<leg_root.foot_traj_mapping_to_body<<std::endl;

        joi_des_pos_vice.block<3,1>(0,5)=_kinematic[5].invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,5));
        joi_des_pos_vice.block<3,1>(0,4)=_kinematic[4].invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,4));
        joi_des_pos_vice.block<3,1>(0,3)=_kinematic[3].invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,3));
        joi_des_pos_vice.block<3,1>(0,2)=_kinematic[2].invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,2));
        joi_des_pos_vice.block<3,1>(0,1)=_kinematic[1].invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,1));
        joi_des_pos_vice.block<3,1>(0,0)=_kinematic[0].invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,0));

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
