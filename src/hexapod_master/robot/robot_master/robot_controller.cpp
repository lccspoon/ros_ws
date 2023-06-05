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
Eigen::Matrix<double,18,1> CorrinvDesangle,invDesangle ;
void Hexapod::trajectoryPlaning(void) 
{
        leg_root.foot_swing_traj.block<3,1>(0,5)=_RbTraj.bezierCurve(cpg_scheduler(0,0),cpg_scheduler(1,0) )*_Rb._FooBodAdjMap.step_amplitude;
        leg_root.foot_swing_traj.block<3,1>(0,4)=_RmTraj.bezierCurve(cpg_scheduler(0,1),cpg_scheduler(1,1) )*_Rm._FooBodAdjMap.step_amplitude;
        leg_root.foot_swing_traj.block<3,1>(0,3)=_RfTraj.bezierCurve(cpg_scheduler(0,2),cpg_scheduler(1,2) )*_Rf._FooBodAdjMap.step_amplitude;
        leg_root.foot_swing_traj.block<3,1>(0,2)=_LbTraj.bezierCurve(cpg_scheduler(0,5),cpg_scheduler(1,5) )*_Lb._FooBodAdjMap.step_amplitude;
        leg_root.foot_swing_traj.block<3,1>(0,1)=_LmTraj.bezierCurve(cpg_scheduler(0,4),cpg_scheduler(1,4) )*_Lm._FooBodAdjMap.step_amplitude;
        leg_root.foot_swing_traj.block<3,1>(0,0)=_LfTraj.bezierCurve(cpg_scheduler(0,3),cpg_scheduler(1,3) )*_Lf._FooBodAdjMap.step_amplitude;

        //lcc 20230513: 腿默认顺序为: lf lm lb rf rm rb -> 1 2 3 4 5 6
        cpg_touch_down_scheduler<<cpg_scheduler(1,3), cpg_scheduler(1,4), cpg_scheduler(1,5), cpg_scheduler(1,2), cpg_scheduler(1,1), cpg_scheduler(1,0);

        leg_root.foot_traj_mapping_to_body.block<3,1>(0,5)=_Rb._FooBodAdjMap.trajAndAdjustMapping(leg_root.foot_swing_traj.block<3,1>(0,5));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,4)=_Rm._FooBodAdjMap.trajAndAdjustMapping(leg_root.foot_swing_traj.block<3,1>(0,4));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,3)=_Rf._FooBodAdjMap.trajAndAdjustMapping(leg_root.foot_swing_traj.block<3,1>(0,3));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,2)=_Lb._FooBodAdjMap.trajAndAdjustMapping(leg_root.foot_swing_traj.block<3,1>(0,2));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,1)=_Lm._FooBodAdjMap.trajAndAdjustMapping(leg_root.foot_swing_traj.block<3,1>(0,1));
        leg_root.foot_traj_mapping_to_body.block<3,1>(0,0)=_Lf._FooBodAdjMap.trajAndAdjustMapping(leg_root.foot_swing_traj.block<3,1>(0,0));
        leg_root.foot_traj_mapping_to_body(1,3)=-leg_root.foot_traj_mapping_to_body(1,3);
        leg_root.foot_traj_mapping_to_body(1,4)=-leg_root.foot_traj_mapping_to_body(1,4);
        leg_root.foot_traj_mapping_to_body(1,5)=-leg_root.foot_traj_mapping_to_body(1,5);
        // std::cout<<"leg_root.foot_traj_mapping_to_body: "<<std::endl;
        // std::cout<<leg_root.foot_traj_mapping_to_body<<std::endl;

        joi_des_pos_vice.block<3,1>(0,5)=_RbKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,5));
        joi_des_pos_vice.block<3,1>(0,4)=_RmKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,4));
        joi_des_pos_vice.block<3,1>(0,3)=_RfKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,3));
        joi_des_pos_vice.block<3,1>(0,2)=_LbKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,2));
        joi_des_pos_vice.block<3,1>(0,1)=_LmKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,1));
        joi_des_pos_vice.block<3,1>(0,0)=_LfKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,0));
        // 话题画曲线
        invDesangle.block<3,1>(0,0)=_LfKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,0));
        invDesangle.block<3,1>(3,0)=_LmKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,1));
        invDesangle.block<3,1>(6,0)=_LbKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,2));
        invDesangle.block<3,1>(9,0)=_RfKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,3));
        invDesangle.block<3,1>(12,0)=_RmKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,4));
        invDesangle.block<3,1>(15,0)=_RbKin.invKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,5));
      

        //zyt 20230518 足端修正

        // joi_des_pos_vice.block<3,1>(0,5)=_RbCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,5));
        // joi_des_pos_vice.block<3,1>(0,4)=_RmCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,4));
        // joi_des_pos_vice.block<3,1>(0,3)= _RfCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,3));
        // joi_des_pos_vice.block<3,1>(0,2)=_LbCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,2));
        // joi_des_pos_vice.block<3,1>(0,1)= _LmCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,1));
        // joi_des_pos_vice.block<3,1>(0,0)= _LfCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,0));
        /*.........话题画曲线传参数..............*/
        CorrinvDesangle.block<3,1>(5,0)=_RbCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,5));
        CorrinvDesangle.block<3,1>(4,0)=_RmCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,4));
        CorrinvDesangle.block<3,1>(3,0)=_RfCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,3));
        CorrinvDesangle.block<3,1>(2,0)=_LbCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,2));
        CorrinvDesangle.block<3,1>(1,0)=_LmCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,1));
        CorrinvDesangle.block<3,1>(0,0)=_LfCorrKin.CorrinvKinematic(leg_root.foot_traj_mapping_to_body.block<3,1>(0,0));

    //lcc 20230405:      以下程序让机器人能在启动时，读去当前角度并且从当前角度缓慢启动到站立姿态
    #if HARD_WARE==1
            joi_des_pos_vice.block<3,1>(0,0)=-joi_des_pos_vice.block<3,1>(0,0)+HarBri.lf_off_set;
            joi_des_pos_vice.block<3,1>(0,1)=-joi_des_pos_vice.block<3,1>(0,1)+HarBri.lm_off_set;
            joi_des_pos_vice.block<3,1>(0,2)=-joi_des_pos_vice.block<3,1>(0,2)+HarBri.lb_off_set;
            joi_des_pos_vice.block<3,1>(0,3)=joi_des_pos_vice.block<3,1>(0,3)+HarBri.rf_off_set;
            joi_des_pos_vice.block<3,1>(0,4)=joi_des_pos_vice.block<3,1>(0,4)+HarBri.rm_off_set;
            joi_des_pos_vice.block<3,1>(0,5)=joi_des_pos_vice.block<3,1>(0,5)+HarBri.rb_off_set;
    #elif HARD_WARE==2

    #endif
    motor_pos_set_count=800;
    if(get_joint_pos_flag==true && joint_pos_run_count!=motor_pos_set_count)
    {
        leg_root.joint_des_pos.block<3,1>(0,0)=_LfStaPosLinTrans.linearConvert(leg_root.joint_des_pos.block<3,1>(0,0), joi_des_pos_vice.block<3,1>(0,0), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,1)=_LmStaPosLinTrans.linearConvert(leg_root.joint_des_pos.block<3,1>(0,1), joi_des_pos_vice.block<3,1>(0,1), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,2)=_LbStaPosLinTrans.linearConvert(leg_root.joint_des_pos.block<3,1>(0,2), joi_des_pos_vice.block<3,1>(0,2), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,3)=_RfStaPosLinTrans.linearConvert(leg_root.joint_des_pos.block<3,1>(0,3), joi_des_pos_vice.block<3,1>(0,3), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,4)=_RmStaPosLinTrans.linearConvert(leg_root.joint_des_pos.block<3,1>(0,4), joi_des_pos_vice.block<3,1>(0,4), motor_pos_set_count);
        leg_root.joint_des_pos.block<3,1>(0,5)=_RbStaPosLinTrans.linearConvert(leg_root.joint_des_pos.block<3,1>(0,5), joi_des_pos_vice.block<3,1>(0,5), motor_pos_set_count);
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
        leg_root.joint_des_pos.block<3,1>(0,5)=_RbDatUnuProtect.sendDataConPro(6,leg_root.joint_des_pos.block<3,1>(0,5),5*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,4)=_RmDatUnuProtect.sendDataConPro(5,leg_root.joint_des_pos.block<3,1>(0,4),5*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,3)=_RfDatUnuProtect.sendDataConPro(4,leg_root.joint_des_pos.block<3,1>(0,3),5*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,2)=_LbDatUnuProtect.sendDataConPro(3,leg_root.joint_des_pos.block<3,1>(0,2),5*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,1)=_LmDatUnuProtect.sendDataConPro(2,leg_root.joint_des_pos.block<3,1>(0,1),5*_RAD1);
        leg_root.joint_des_pos.block<3,1>(0,0)=_LfDatUnuProtect.sendDataConPro(1,leg_root.joint_des_pos.block<3,1>(0,0),5*_RAD1);

    }

    //lcc 20230405: 对joi_des_pos的连续性进行保护时，但凡有一个电机的joi_des_pos的相邻两次角度差值过大，所有电机都会stop
    if(_RbDatUnuProtect.StopFlag==false or _RmDatUnuProtect.StopFlag==false or _RfDatUnuProtect.StopFlag==false
    or _LbDatUnuProtect.StopFlag==false or _LmDatUnuProtect.StopFlag==false or _LfDatUnuProtect.StopFlag==false)
    {
        _RbDatUnuProtect.StopFlag=false;_RmDatUnuProtect.StopFlag=false;_RfDatUnuProtect.StopFlag=false;
        _LbDatUnuProtect.StopFlag=false;_LmDatUnuProtect.StopFlag=false;_LfDatUnuProtect.StopFlag=false;
    }
}

void Hexapod::positionController(void)
{
        leg_root.joint_msg_pub=leg_root.joint_des_pos;
}

void Hexapod::adaController(void)//zyt_adacontrol 2023.4.19
{
    

    // double ada_a=0.05,ada_b=15,ada_beta=0.05; 
    // double kp1=25,kp2=35,kp3=30;
    // double lkd1=1.5,lkd2=2.5,lkd3=2;
    // double rkd1=-1.5,rkd2=-2.5,rkd3=-2;

    double ada_a=0.01,ada_b=15,ada_beta=0.05; 
    double kp1=80,kp2=80,kp3=80;
    double lkd1=10,lkd2=10,lkd3=10;
    double rkd1=-10,rkd2=-10,rkd3=-10;

    Eigen::Matrix<double, 3, 1> matrix;
    matrix.setZero();

    _LfAdaCtrl.setAda(ada_beta,ada_a,ada_b);_LfAdaCtrl.setKp(kp1,kp2,kp3);_LfAdaCtrl.setKd(lkd1,lkd2,lkd3);
    _LmAdadCtrl.setAda(ada_beta,ada_a,ada_b);_LmAdadCtrl.setKp(kp1,kp2,kp3);_LmAdadCtrl.setKd(lkd1,lkd2,lkd3);
    _LbAdaCtrl.setAda(ada_beta,ada_a,ada_b);_LbAdaCtrl.setKp(kp1,kp2,kp3);_LbAdaCtrl.setKd(lkd1,lkd2,lkd3);

    _RfAdaCtrl.setAda(-ada_beta,ada_a,ada_b);_RfAdaCtrl.setKp(kp1,kp2,kp3);_RfAdaCtrl.setKd(rkd1,rkd2,rkd3);
    _RmAdaCtrl.setAda(-ada_beta,ada_a,ada_b);_RmAdaCtrl.setKp(kp1,kp2,kp3);_RmAdaCtrl.setKd(rkd1,rkd2,rkd3);
    _RbAdaCtrl.setAda(-ada_beta,ada_a,ada_b);_RbAdaCtrl.setKp(kp1,kp2,kp3);_RbAdaCtrl.setKd(rkd1,rkd2,rkd3);

    leg_root.joint_des_tor.block<3,1>(0,0)=
                _LfAdaCtrl.adaTau(leg_root.joint_rec_pos.block<1,3>(0,0).transpose(), leg_root.joint_des_pos.block<3,1>(0,0), leg_root.joint_rec_vel.block<1,3>(0,0).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,1)=
                _LmAdadCtrl.adaTau(leg_root.joint_rec_pos.block<1,3>(0,3).transpose(), leg_root.joint_des_pos.block<3,1>(0,1), leg_root.joint_rec_vel.block<1,3>(0,3).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,2)=
                _LbAdaCtrl.adaTau(leg_root.joint_rec_pos.block<1,3>(0,6).transpose(), leg_root.joint_des_pos.block<3,1>(0,2), leg_root.joint_rec_vel.block<1,3>(0,6).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,3)=
                _RfAdaCtrl.adaTau(leg_root.joint_rec_pos.block<1,3>(0,9).transpose(), leg_root.joint_des_pos.block<3,1>(0,3), leg_root.joint_rec_vel.block<1,3>(0,9).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,4)=
                _RmAdaCtrl.adaTau(leg_root.joint_rec_pos.block<1,3>(0,12).transpose(), leg_root.joint_des_pos.block<3,1>(0,4), leg_root.joint_rec_vel.block<1,3>(0,12).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,5)=
                _RbAdaCtrl.adaTau(leg_root.joint_rec_pos.block<1,3>(0,15).transpose(), leg_root.joint_des_pos.block<3,1>(0,5), leg_root.joint_rec_vel.block<1,3>(0,15).transpose(), matrix);

    leg_root.joint_msg_pub=leg_root.joint_des_tor;
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
    // // cout<<"toque:"<<_Rm.f.joi_des_tor[1]<<endl;

   
    // cout<<"toque:"<<_Rm.f.joi_des_tor[1]<<endl;
    
}

void Hexapod::pdController(void)
{
     Eigen::Matrix<double, 3, 1> matrix;
    matrix.setZero();
    // _LfPdCtrl.setKp(2,4,2);_LfPdCtrl.setKd(0.2,0.4,0.2);_LfPdCtrl.setKi(0,0,0);
    // _LmPdCtrl.setKp(2,4,2);_LmPdCtrl.setKd(0.2,0.4,0.2);_LmPdCtrl.setKi(0,0,0);
    // _LbPdCtrl.setKp(2,4,2);_LbPdCtrl.setKd(0.2,0.4,0.2);_LbPdCtrl.setKi(0,0,0);

    // _RfPdCtrl.setKp(2,4,2);_RfPdCtrl.setKd(-0.2,-0.4,-0.2);_RfPdCtrl.setKi(0,0,0);
    // _RmPdCtrl.setKp(2,4,2);_RmPdCtrl.setKd(-0.2,-0.4,-0.2);_RmPdCtrl.setKi(0,0,0);
    // _RbPdCtrl.setKp(2,4,2);_RbPdCtrl.setKd(-0.2,-0.4,-0.2);_RbPdCtrl.setKi(0,0,0);


    //pdcontrol zyt 20230517 
    double lkp1=80,lkp2=80,lkp3=80;
    double lkd1= 10,lkd2=10,lkd3=10;
    double rkp1=80,rkp2=80,rkp3=80;
    double rkd1=-10,rkd2=-10,rkd3=-10;
    // _LfPdCtrl.setKp(150,220,200);_LfPdCtrl.setKd(10,14,12);//_LfPdCtrl.setKi(0,0,0);
    // _LmPdCtrl.setKp(150,220,200);_LmPdCtrl.setKd(10,14,12);//_LmPdCtrl.setKi(0,0,0);
    // _LbPdCtrl.setKp(150,220,200);_LbPdCtrl.setKd(10,14,12);//_LbPdCtrl.setKi(0,0,0);

    // _RfPdCtrl.setKp(150,220,200);_RfPdCtrl.setKd(-10,-14,-12);//_RfPdCtrl.setKi(0,0,0);
    // _RmPdCtrl.setKp(150,220,200);_RmPdCtrl.setKd(-10,-14,-12);//_RmPdCtrl.setKi(0,0,0);
    // _RbPdCtrl.setKp(150,220,200);_RbPdCtrl.setKd(-10,-14,-12);//_RbPdCtrl.setKi(0,0,0);

    _LfPdCtrl.setKp(lkp1,lkp2,lkp3);_LfPdCtrl.setKd(lkd1,lkd2,lkd3);//_LfPdCtrl.setKi(0,0,0);
    _LmPdCtrl.setKp(lkp1,lkp2,lkp3);_LmPdCtrl.setKd(lkd1,lkd2,lkd3);//_LmPdCtrl.setKi(0,0,0);
    _LbPdCtrl.setKp(lkp1,lkp2,lkp3);_LbPdCtrl.setKd(lkd1,lkd2,lkd3);//_LbPdCtrl.setKi(0,0,0);

    _RfPdCtrl.setKp(rkp1,rkp2,rkp3);_RfPdCtrl.setKd(rkd1,rkd2,rkd3);//_RfPdCtrl.setKi(0,0,0);
    _RmPdCtrl.setKp(rkp1,rkp2,rkp3);_RmPdCtrl.setKd(rkd1,rkd2,rkd3);//_RmPdCtrl.setKi(0,0,0);
    _RbPdCtrl.setKp(rkp1,rkp2,rkp3);_RbPdCtrl.setKd(rkd1,rkd2,rkd3);//_RbPdCtrl.setKi(0,0,0);

    leg_root.joint_des_tor.block<3,1>(0,0)=
                _LfPdCtrl.pdTau(leg_root.joint_rec_pos.block<1,3>(0,0).transpose(), leg_root.joint_des_pos.block<3,1>(0,0), leg_root.joint_rec_vel.block<1,3>(0,0).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,1)=
                _LmPdCtrl.pdTau(leg_root.joint_rec_pos.block<1,3>(0,3).transpose(), leg_root.joint_des_pos.block<3,1>(0,1), leg_root.joint_rec_vel.block<1,3>(0,3).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,2)=
                _LbPdCtrl.pdTau(leg_root.joint_rec_pos.block<1,3>(0,6).transpose(), leg_root.joint_des_pos.block<3,1>(0,2), leg_root.joint_rec_vel.block<1,3>(0,6).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,3)=
                _RfPdCtrl.pdTau(leg_root.joint_rec_pos.block<1,3>(0,9).transpose(), leg_root.joint_des_pos.block<3,1>(0,3), leg_root.joint_rec_vel.block<1,3>(0,9).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,4)=
                _RmPdCtrl.pdTau(leg_root.joint_rec_pos.block<1,3>(0,12).transpose(), leg_root.joint_des_pos.block<3,1>(0,4), leg_root.joint_rec_vel.block<1,3>(0,12).transpose(), matrix);
    leg_root.joint_des_tor.block<3,1>(0,5)=
                _RbPdCtrl.pdTau(leg_root.joint_rec_pos.block<1,3>(0,15).transpose(), leg_root.joint_des_pos.block<3,1>(0,5), leg_root.joint_rec_vel.block<1,3>(0,15).transpose(), matrix);


     //pidcontrol 20230517 zyt
    // _LfPdCtrl.setKp(70,70,70);_LfPdCtrl.setKd(500,500,500);_LfPdCtrl.setKi(0.03,0.03,0.03);
    // _LmPdCtrl.setKp(70,70,70);_LmPdCtrl.setKd(500,500,500);_LmPdCtrl.setKi(0.03,0.03,0.03);
    // _LbPdCtrl.setKp(70,70,70);_LbPdCtrl.setKd(500,500,500);_LbPdCtrl.setKi(0.03,0.03,0.03);

    // _RfPdCtrl.setKp(70,70,70);_RfPdCtrl.setKd(500,500,500);_RfPdCtrl.setKi(0.03,0.03,0.03);
    // _RmPdCtrl.setKp(70,70,70);_RmPdCtrl.setKd(500,500,500);_RmPdCtrl.setKi(0.03,0.03,0.03);
    // _RbPdCtrl.setKp(70,70,70);_RbPdCtrl.setKd(500,500,500);_RbPdCtrl.setKi(0.03,0.03,0.03);

   
    // leg_root.joint_des_tor.block<3,1>(0,0)=
    //             _LfPdCtrl.positonPidTau(leg_root.joint_rec_pos.block<1,3>(0,0).transpose(), leg_root.joint_des_pos.block<3,1>(0,0), leg_root.joint_rec_vel.block<1,3>(0,0).transpose(), matrix);
    // leg_root.joint_des_tor.block<3,1>(0,1)=
    //             _LmPdCtrl.positonPidTau(leg_root.joint_rec_pos.block<1,3>(0,3).transpose(), leg_root.joint_des_pos.block<3,1>(0,1), leg_root.joint_rec_vel.block<1,3>(0,3).transpose(), matrix);
    // leg_root.joint_des_tor.block<3,1>(0,2)=
    //             _LbPdCtrl.positonPidTau(leg_root.joint_rec_pos.block<1,3>(0,6).transpose(), leg_root.joint_des_pos.block<3,1>(0,2), leg_root.joint_rec_vel.block<1,3>(0,6).transpose(), matrix);
    // leg_root.joint_des_tor.block<3,1>(0,3)=
    //             _RfPdCtrl.positonPidTau(leg_root.joint_rec_pos.block<1,3>(0,9).transpose(), leg_root.joint_des_pos.block<3,1>(0,3), leg_root.joint_rec_vel.block<1,3>(0,9).transpose(), matrix);
    // leg_root.joint_des_tor.block<3,1>(0,4)=
    //             _RmPdCtrl.positonPidTau(leg_root.joint_rec_pos.block<1,3>(0,12).transpose(), leg_root.joint_des_pos.block<3,1>(0,4), leg_root.joint_rec_vel.block<1,3>(0,12).transpose(), matrix);
    // leg_root.joint_des_tor.block<3,1>(0,5)=
    //             _RbPdCtrl.positonPidTau(leg_root.joint_rec_pos.block<1,3>(0,15).transpose(), leg_root.joint_des_pos.block<3,1>(0,5), leg_root.joint_rec_vel.block<1,3>(0,15).transpose(), matrix);

    
    leg_root.joint_msg_pub=leg_root.joint_des_tor;
    


}

//20230519 zyt 机身控制
void Hexapod::wbcController(void)
{
    Vec12<double> x1_ref,_xact;//Eigen::Matrix<double,12,1>
    Vec18<double> _fpos,MPCforce;
    Vec6<int> TDsignal;

    //期望位置在速度 期望角度 期望角速度传参 
    //足端位置 支撑信号传参
    MPCforce.setZero();
    x1_ref.setZero();
    _xact.setZero();
    _fpos << 1,-1,1, 0,-1,-1, -1,-1,-1,
            -1,1,-1, 0,1,-1, 1,1,-1;
    TDsignal<< 1,0,1,0,1,0;
    // TDsignal.setOnes();

    _QPforce.initialize();//初始化

    _QPforce.Setfeedforward(MPCforce);//暂时用不到
    _QPforce.SetContactData(TDsignal);//CPG摆动\支撑
    _QPforce.set_desiredTrajectoryData(x1_ref);//设置期望值 xyz vxyz rpy vrpy
    _QPforce.updateProblemData(_xact,_fpos);//设置反馈值xyz vxyz rpy vrpy和足端位置(机身坐标系下的) 

    // _QPforce.QPforce  //结果 18*1
    // std::cout<< _QPforce.QPforce.block(0,0,3,1).transpose()<<"\n";
    // std::cout<< _QPforce.QPforce.block(3,0,3,1).transpose()<<"\n";
    // std::cout<< _QPforce.QPforce.block(6,0,3,1).transpose()<<"\n";
    // std::cout<< _QPforce.QPforce.block(9,0,3,1).transpose()<<"\n";
    // std::cout<< _QPforce.QPforce.block(12,0,3,1).transpose()<<"\n";
    // std::cout<< _QPforce.QPforce.block(15,0,3,1).transpose()<<"\n";
    //     double a;
    //     a=0;
    // for (int i = 0; i < 6; i++)
    // {
    //     a+=_QPforce.QPforce(2+3*i);
        
    // }
    // std::cout<<a<<"\n";
  
}