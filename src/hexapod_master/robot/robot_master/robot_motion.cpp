#include"robot_motion.h"


void Hexapod::setSixLegSteAmplitude(double lf_step_amplitude,double lm_step_amplitude,double lb_step_amplitude,
                            double rf_step_amplitude,double rm_step_amplitude,double rb_step_amplitude,double switch_period)
{
    _FooBodAdjMap[5].step_amplitude=_FooBodAdjMap[5].SteAmpLinTran.linearConvert(_FooBodAdjMap[5].step_amplitude,rb_step_amplitude,switch_period);
    _FooBodAdjMap[4].step_amplitude=_FooBodAdjMap[4].SteAmpLinTran.linearConvert(_FooBodAdjMap[4].step_amplitude,rm_step_amplitude,switch_period);
    _FooBodAdjMap[3].step_amplitude=_FooBodAdjMap[3].SteAmpLinTran.linearConvert(_FooBodAdjMap[3].step_amplitude,rf_step_amplitude,switch_period);
    _FooBodAdjMap[2].step_amplitude=_FooBodAdjMap[2].SteAmpLinTran.linearConvert(_FooBodAdjMap[2].step_amplitude,lb_step_amplitude,switch_period);
    _FooBodAdjMap[1].step_amplitude=_FooBodAdjMap[1].SteAmpLinTran.linearConvert(_FooBodAdjMap[1].step_amplitude,lm_step_amplitude,switch_period);
    _FooBodAdjMap[0].step_amplitude=_FooBodAdjMap[0].SteAmpLinTran.linearConvert(_FooBodAdjMap[0].step_amplitude,lf_step_amplitude,switch_period);

}
void Hexapod::setSixLegRzz(double lf_rzz,double lm_rzz,double lb_rzz,
                    double rf_rzz,double rm_rzz,double rb_rzz,double switch_period)    //lcc 用于旋转足端轨迹平面
{
    _FooBodAdjMap[5].rzz=_FooBodAdjMap[5].RzzLinTran.linearConvert(_FooBodAdjMap[5].rzz,rb_rzz*_RAD1,switch_period);
    _FooBodAdjMap[4].rzz=_FooBodAdjMap[4].RzzLinTran.linearConvert(_FooBodAdjMap[4].rzz,rm_rzz*_RAD1,switch_period);
    _FooBodAdjMap[3].rzz=_FooBodAdjMap[3].RzzLinTran.linearConvert(_FooBodAdjMap[3].rzz,rf_rzz*_RAD1,switch_period);
    _FooBodAdjMap[2].rzz=_FooBodAdjMap[2].RzzLinTran.linearConvert(_FooBodAdjMap[2].rzz,lb_rzz*_RAD1,switch_period);
    _FooBodAdjMap[1].rzz=_FooBodAdjMap[1].RzzLinTran.linearConvert(_FooBodAdjMap[1].rzz,lm_rzz*_RAD1,switch_period);
    _FooBodAdjMap[0].rzz=_FooBodAdjMap[0].RzzLinTran.linearConvert(_FooBodAdjMap[0].rzz,lf_rzz*_RAD1,switch_period);

}
void Hexapod::robTripleWalk()
{
    _Cpg.gait=_Cpg.GaitLinTran.linearConvert(_Cpg.gait,0.5,400); 
}
void Hexapod::robQuadruWalk()
{
    _Cpg.gait=_Cpg.GaitLinTran.linearConvert(_Cpg.gait,0.666,400); 
}
void Hexapod::robHexapodWalk()
{
    _Cpg.gait=_Cpg.GaitLinTran.linearConvert(_Cpg.gait,0.833,400); 
}

Eigen::Vector3d static_pos;
void Hexapod::setStandPose(void)
{
    #if HARD_WARE==1
        static_pos<< 6.5*0.01, 9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,0)=static_pos;

        static_pos<< 3.75*0.01, 9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,1)=static_pos;

        static_pos<< 1.5*0.01, 9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,2)=static_pos;

        static_pos<< 6.5*0.01, -9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,3)=static_pos;

        static_pos<< 3.75*0.01, -9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,4)=static_pos;

        static_pos<< 1.5*0.01, -9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,5)=static_pos;
    #elif HARD_WARE==2
        static_pos<< 6.5*0.01, 9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,0)=static_pos;

        static_pos<< 4.25*0.01, 9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,1)=static_pos;

        static_pos<< 2*0.01, 9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,2)=static_pos;

        static_pos<< 6.5*0.01, -9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,3)=static_pos;

        static_pos<< 4.25*0.01, -9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,4)=static_pos;

        static_pos<< 2*0.01, -9*0.01, -13.5*0.01;
        leg_root.foot_set_static_pos.block<3,1>(0,5)=static_pos;
    #endif
    // test 
    // float x=7.3, y=18, z=17;

    // static_pos<< x*0.01, y*0.01, -z*0.01;
    // leg_root.foot_set_static_pos.block<3,1>(0,0)=static_pos;
    // leg_root.foot_set_static_pos.block<3,1>(0,1)=static_pos;
    // leg_root.foot_set_static_pos.block<3,1>(0,2)=static_pos;

    // static_pos<< x*0.01, -y*0.01, -z*0.01;
    // leg_root.foot_set_static_pos.block<3,1>(0,3)=static_pos;
    // leg_root.foot_set_static_pos.block<3,1>(0,4)=static_pos;
    // leg_root.foot_set_static_pos.block<3,1>(0,5)=static_pos;
}


void Hexapod::robStand(double switch_period)
{
    setStandPose();

    leg_root.foot_static_pos.block<3,1>(0,0)=_set_static_pos_conver[0].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,0),
    leg_root.foot_set_static_pos.block<3,1>(0,0),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,1)=_set_static_pos_conver[1].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,1),
    leg_root.foot_set_static_pos.block<3,1>(0,1),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,2)=_set_static_pos_conver[2].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,2),
    leg_root.foot_set_static_pos.block<3,1>(0,2),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,3)=_set_static_pos_conver[3].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,3),
    leg_root.foot_set_static_pos.block<3,1>(0,3),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,4)=_set_static_pos_conver[4].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,4),
    leg_root.foot_set_static_pos.block<3,1>(0,4),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,5)=_set_static_pos_conver[5].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,5),
    leg_root.foot_set_static_pos.block<3,1>(0,5),switch_period);

    setSixLegSteAmplitude(0,0,0,  0,0,0,  400);
    setSixLegRzz(0,0,0,  0,0,0,  400);
}
void Hexapod::robSquat(double switch_period)
{

    static_pos<< 7.3*0.01, 11.5*0.01, -5*0.01;
    leg_root.foot_set_static_pos.block<3,1>(0,0)=static_pos;
    static_pos<< 6.3*0.01, 11.5*0.01, -5*0.01;
    leg_root.foot_set_static_pos.block<3,1>(0,1)=static_pos;
    static_pos<< 5.3*0.01, 11.5*0.01, -5*0.01;
    leg_root.foot_set_static_pos.block<3,1>(0,2)=static_pos;
    static_pos<< 7.3*0.01, -11.5*0.01, -5*0.01;
    leg_root.foot_set_static_pos.block<3,1>(0,3)=static_pos;
    static_pos<< 6.3*0.01, -11.5*0.01, -5*0.01;
    leg_root.foot_set_static_pos.block<3,1>(0,4)=static_pos;
    static_pos<< 5.3*0.01, -11.5*0.01, -5*0.01;
    leg_root.foot_set_static_pos.block<3,1>(0,5)=static_pos;

    leg_root.foot_static_pos.block<3,1>(0,0)=_set_static_pos_conver[0].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,0),
    leg_root.foot_set_static_pos.block<3,1>(0,0),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,1)=_set_static_pos_conver[1].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,1),
    leg_root.foot_set_static_pos.block<3,1>(0,1),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,2)=_set_static_pos_conver[2].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,2),
    leg_root.foot_set_static_pos.block<3,1>(0,2),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,3)=_set_static_pos_conver[3].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,3),
    leg_root.foot_set_static_pos.block<3,1>(0,3),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,4)=_set_static_pos_conver[4].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,4),
    leg_root.foot_set_static_pos.block<3,1>(0,4),switch_period);

    leg_root.foot_static_pos.block<3,1>(0,5)=_set_static_pos_conver[5].linearConvert(
    leg_root.foot_static_pos.block<3,1>(0,5),
    leg_root.foot_set_static_pos.block<3,1>(0,5),switch_period);

    setSixLegSteAmplitude(0,0,0,  0,0,0,  400);
    setSixLegRzz(0,0,0,  0,0,0,  400);
}
void Hexapod::robGoAhead(double amplitude)
{
    setStandPose();

    setSixLegSteAmplitude(amplitude,amplitude,amplitude,  amplitude,amplitude,amplitude,  400);
    setSixLegRzz(0,0,0,  0,0,0,  400);
}
void Hexapod::robGoBack(double amplitude)
{
    setSixLegSteAmplitude(amplitude,amplitude,amplitude,  amplitude,amplitude,amplitude,  400);
    setSixLegRzz(180,180,180,  180,180,180,  400);
}
void Hexapod::robGoLeft(double amplitude)
{

    setSixLegSteAmplitude(amplitude,amplitude,amplitude,  amplitude,amplitude,amplitude,  400);
    setSixLegRzz(-90,-90,-90,  -90,-90,-90,  400);
}
void Hexapod::robGoRight(double amplitude)
{

    setSixLegSteAmplitude(amplitude,amplitude,amplitude,  amplitude,amplitude,amplitude,  400);
    setSixLegRzz(90,90,90,  90,90,90,  400);
}
void Hexapod::robTurnRightAround(double amplitude)
{

    setSixLegSteAmplitude(amplitude,amplitude,amplitude,  amplitude,amplitude,amplitude,  400);
    setSixLegRzz(180,180,180,  0,0,0,  400);
}
void Hexapod::robTurnLeftAround(double amplitude)
{

    setSixLegSteAmplitude(amplitude,amplitude,amplitude,  amplitude,amplitude,amplitude,  400);
    setSixLegRzz(0,0,0,  180,180,180,  400);
}

int aaa=0;
void Hexapod::keyBoardControl(int key_value)
{
    if(motor_pos_ach_count_flag==1)  //lcc 20230428:　启动动作完成后， motor_pos_ach_count_flag=1
    {
            switch (key_value)
            {
                    case 'w':
                            {      
                                    if( _set_static_pos_conver[0].retConvDoneFlag() && 
                                        _set_static_pos_conver[1].retConvDoneFlag() && 
                                        _set_static_pos_conver[2].retConvDoneFlag() && 
                                        _set_static_pos_conver[3].retConvDoneFlag() && 
                                        _set_static_pos_conver[4].retConvDoneFlag() && 
                                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag )
                                            robGoAhead(0.45);
                                        // movement_mode=1;   //
                                        KEYBOARD_CONTINUE_MODE=1;
                                        // printf("\n wwwwwwwwwwwwwwwwwwwwww \n");
                            }
                            break;
                    case 's':
                            {
                                    if( _set_static_pos_conver[0].retConvDoneFlag() && 
                                        _set_static_pos_conver[1].retConvDoneFlag() && 
                                        _set_static_pos_conver[2].retConvDoneFlag() && 
                                        _set_static_pos_conver[3].retConvDoneFlag() && 
                                        _set_static_pos_conver[4].retConvDoneFlag() && 
                                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag )
                                            robGoBack(0.45);
                                        KEYBOARD_CONTINUE_MODE=1;
                                        // printf("\n ssssssssssssssssssssssssssssss \n");
                                        
                            }
                            break;
                    case 'a':
                            {
                                    if( _set_static_pos_conver[0].retConvDoneFlag() && 
                                        _set_static_pos_conver[1].retConvDoneFlag() && 
                                        _set_static_pos_conver[2].retConvDoneFlag() && 
                                        _set_static_pos_conver[3].retConvDoneFlag() && 
                                        _set_static_pos_conver[4].retConvDoneFlag() && 
                                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag )
                                            robGoLeft(0.45);
                                        KEYBOARD_CONTINUE_MODE=1;
                                        // printf("\n ssssssssssssssssssssssssssssss \n");
                            }
                            break;
                    case 'd':
                            {
                                    if( _set_static_pos_conver[0].retConvDoneFlag() && 
                                        _set_static_pos_conver[1].retConvDoneFlag() && 
                                        _set_static_pos_conver[2].retConvDoneFlag() && 
                                        _set_static_pos_conver[3].retConvDoneFlag() && 
                                        _set_static_pos_conver[4].retConvDoneFlag() && 
                                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag )
                                            robGoRight(0.45);
                                        KEYBOARD_CONTINUE_MODE=1;
                                        
                            }
                            break;
                    case 'q':
                            {
                                    if( _set_static_pos_conver[0].retConvDoneFlag() && 
                                        _set_static_pos_conver[1].retConvDoneFlag() && 
                                        _set_static_pos_conver[2].retConvDoneFlag() && 
                                        _set_static_pos_conver[3].retConvDoneFlag() && 
                                        _set_static_pos_conver[4].retConvDoneFlag() && 
                                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag )
                                            robTurnRightAround(0.45);
                                        KEYBOARD_CONTINUE_MODE=1;
                            }
                            break;
                    case 'e':
                            {
                                    if( _set_static_pos_conver[0].retConvDoneFlag() && 
                                        _set_static_pos_conver[1].retConvDoneFlag() && 
                                        _set_static_pos_conver[2].retConvDoneFlag() && 
                                        _set_static_pos_conver[3].retConvDoneFlag() && 
                                        _set_static_pos_conver[4].retConvDoneFlag() && 
                                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag )
                                            robTurnLeftAround(0.45);
                                        KEYBOARD_CONTINUE_MODE=1;
                            }
                            break;
                    case '1':
                            {
                                    if( _set_static_pos_conver[0].retConvDoneFlag() && 
                                        _set_static_pos_conver[1].retConvDoneFlag() && 
                                        _set_static_pos_conver[2].retConvDoneFlag() && 
                                        _set_static_pos_conver[3].retConvDoneFlag() && 
                                        _set_static_pos_conver[4].retConvDoneFlag() && 
                                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag )
                                            robTripleWalk();
                                        KEYBOARD_CONTINUE_MODE=1;
                            }
                            break;
                    case '2':
                            {
                                    if( _set_static_pos_conver[0].retConvDoneFlag() && 
                                        _set_static_pos_conver[1].retConvDoneFlag() && 
                                        _set_static_pos_conver[2].retConvDoneFlag() && 
                                        _set_static_pos_conver[3].retConvDoneFlag() && 
                                        _set_static_pos_conver[4].retConvDoneFlag() && 
                                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag )
                                            robQuadruWalk();
                                        KEYBOARD_CONTINUE_MODE=1;
                            }
                            break;
                    case '3':
                            {
                                    if( _set_static_pos_conver[0].retConvDoneFlag() && 
                                        _set_static_pos_conver[1].retConvDoneFlag() && 
                                        _set_static_pos_conver[2].retConvDoneFlag() && 
                                        _set_static_pos_conver[3].retConvDoneFlag() && 
                                        _set_static_pos_conver[4].retConvDoneFlag() && 
                                        _set_static_pos_conver[5].retConvDoneFlag() && stand_done_flag )
                                            robHexapodWalk();
                                        KEYBOARD_CONTINUE_MODE=1;
                                        
                            }
                            break;       
                    case 'c':
                            {
                                    robSquat(400);
                                    stand_done_flag=false;
                                    KEYBOARD_CONTINUE_MODE=1;

                            }
                            break;
                    case 'i':     //lcc 20230330:进入闭环
                            {       
                                    #if HARD_WARE==1
                                            // HarBri.robDataLoadEnterCloseLoop();
                                            // HarBri.sendAllLegMsg();     
                                    #endif
                                    KEYBOARD_CONTINUE_MODE=1;
                            }
                            break; 
                    case 'o':    //lcc 20230330:退出闭环
                            {
                                    #if HARD_WARE==1
                                            HarBri.robDataLoadExitCloesLoop();
                                            HarBri.sendAllLegMsg();   
                                    #endif
                                    KEYBOARD_CONTINUE_MODE=1; //lcc : =1 表示一直发送
                            }
                            break;

                    /******************   设置运动参数    ******************/
                    case 'y':
                            {  
                                set_z_deviation=set_z_deviation+1*0.01;
                                printf("set_z_deviation:%f\n",set_z_deviation);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    case 'u':
                            {  
                                set_z_deviation=set_z_deviation-1*0.01;
                                printf("set_z_deviation:%f\n",set_z_deviation);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;

                    case 'h':
                            {  
                                set_step_hight_k=set_step_hight_k+0.1;
                                if(set_step_hight_k<=0) set_step_hight_k=0;
                                printf("set_step_hight_k:%f\n",set_step_hight_k);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    case 'j':
                            {  
                                set_step_hight_k=set_step_hight_k-0.1;
                                if(set_step_hight_k<=0) set_step_hight_k=0;
                                printf("set_step_hight_k:%f\n",set_step_hight_k);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;

                    case 'n':
                            {  
                                set_step_length_k=set_step_length_k+0.1;
                                if(set_step_length_k<=0) set_step_length_k=0;
                                printf("set_step_length_k:%f\n",set_step_length_k);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    case 'm':
                            {  
                                set_step_length_k=set_step_length_k-0.1;
                                if(set_step_length_k<=0) set_step_length_k=0;
                                printf("set_step_length_k:%f\n",set_step_length_k);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    /******************-----------------******************/

                    /******************  设置姿态参数    ******************/
                //     case 'r':   //右键盘－＞　７
                //             {  
                //                 set_yaw=set_yaw+1*_RAD1;
                //                 printf("set_yaw:%f\n",set_yaw);
                //                 KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                //             }
                //             break;

                    case 'r': 
                            {  
                                t_test_key=0;
                            }
                            break;
                    case 't': 
                            {  
                                t_test_key=1;
                            }
                            break;

                    case 'f':   //右键盘－＞ 4　
                            {  
                                set_roll=set_roll+1*_RAD1;
                                printf("set_roll:%f\n",set_roll);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    case 'g':    //右键盘－＞　5
                            {  
                                set_roll=set_roll-1*_RAD1;
                                printf("set_roll:%f\n",set_roll);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    case 'v':   //右键盘－＞ 1　
                            {  
                                set_pitch=set_pitch+1*_RAD1;
                                printf("set_pitch:%f\n",set_pitch);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    case 'b':    //右键盘－＞　2
                            {  
                                set_pitch=set_pitch-1*_RAD1;
                                printf("set_pitch:%f\n",set_pitch);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    /******************－－－－－－－－－－－******************/

                    /******************   设置ＣＰＧ参数   ******************/
                    case ',':    //右键盘－＞　5
                            {  
                                #if HARD_WARE==1
                                        set_cpg_ctrl_cycle=0.001;
                                #elif HARD_WARE==2
                                        set_cpg_ctrl_cycle=0.0010;   //lcc 一个步态周期的点数＝１／set_cpg_ctrl_cycle
                                #endif
                                printf("set_cpg_ctrl_cycle:%f\n",set_cpg_ctrl_cycle);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    case '.':   //右键盘－＞ 1　
                            {  
                                #if HARD_WARE==1
                                set_cpg_ctrl_cycle=0.0025;
                                #elif HARD_WARE==2
                                        set_cpg_ctrl_cycle=0.0015;   //lcc 一个步态周期的点数＝１／set_cpg_ctrl_cycle
                                #endif
                                printf("set_cpg_ctrl_cycle:%f\n",set_cpg_ctrl_cycle);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    case '/':    //右键盘－＞　2
                            {  
                                #if HARD_WARE==1
                                        set_cpg_ctrl_cycle=0.005;
                                #elif HARD_WARE==2
                                        set_cpg_ctrl_cycle=0.002;
                                #endif
                                printf("set_cpg_ctrl_cycle:%f\n",set_cpg_ctrl_cycle);
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                            }
                            break;
                    /******************－－－－－－－－－－－－******************/
                    case 'z':     //lcc 20230502:开启状态估计
                            {  
                                StateEstimator.state_estimator_start_flag=true;
                                KEYBOARD_CONTINUE_MODE=0;    //lcc : =0 表示只发送一次
                                // printf("\n ------------------------- \n");
                            }
                            break;
                    case ' ':
                            {  
                                robStand(400); 
                                stand_done_flag=true;
                                KEYBOARD_CONTINUE_MODE=1;
                                // printf("\n ------------------------- \n");,./
                            }
                            break;
                    case '0':   //lcc 初始设置步长和步高的ｋ值
                            {  
                                set_para_init_flag=1;
                                KEYBOARD_CONTINUE_MODE=0;
                                // printf("\n ------------------------- \n");
                            }
                            break;
                    default:{  
   
                            }
                            break;
            }
    }
}
