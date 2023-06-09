#include"robot_lift_dowm_relex.h"



Eigen::Matrix<double,1,6>  swing_touch_leg_number;//
Eigen::Matrix<double,1,6>  suporting_slip_leg_number;//
// Eigen::Matrix<double,3,6>  foot_lift_traj;
// Eigen::Matrix<double,3,6>  foot_dowm_traj;

int down_stage_flag=0;
int down_stage_switching_flag=0;
int down_stage_count=0;
linear_trans foot_dowmward_traj_res[6],foot_ditch_deepth_est_res[6],foot_dowm_traj_res[6];

int lift_stage_flag=0;
int lift_init_flag=0;
int lift_stage_switching_flag=0;
int lift_stage_count=0;
int swing_traj_switch_flag[6]={0};
linear_trans foot_cross_traj_res[6], foot_lift_traj_res[6], foot_cross_object_est_res[6];

Eigen::Matrix<double,1,6> no_need_changing_dowm_traj, no_need_changing_lift_traj;


Eigen::Matrix<double,1,6>  lift_cpg_phase_last;
Eigen::Matrix<double,1,6>  lift_x_traj_rec;
void Hexapod::liftReaction(void)
{
    //----  以下是抬升反应 ----//
    for (int i = 0; i < 6; i++)
    {
        if(ContactSimple.leg_swingphase_contact_est(i)==1)  //lcc 
        {
            swing_touch_leg_number(i)=1;
        }

         // 如果腿在摆动中触碰，并且　腿抬高轨迹foot_cross_traj　超过了xxcm,那么开始考虑改变足端摆动轨迹
        if( swing_touch_leg_number(i)==1 && fabs( leg_root.foot_cross_traj(2,i) )>1*0.01)  
        {
            swing_traj_switch_flag[i]=1;
        }

        // 调整形状参数
        if(  cpg_touch_down_scheduler(i)==1 && swing_traj_switch_flag[i]==1 ) //如果cpg是摆动
        {
            //调整摆动轨迹，去跨过去障碍
            // lamdaX=[10, -20, 10, 10, 10, 10, 10, 10, 40];
            // lamdaY=[10, 10, 1, 30, 30, 10, 60, 30, 10];
            int tt=int(1/set_cpg_ctrl_cycle)*0.5*0.1;
            int lamdaX_k=2, lamdaY_k=2;
            neur_bezier[i].lamdaX(0)=neur_bezier[i].lamdaX_conver[0].linearConvert( neur_bezier[i].lamdaX(0), 10 * lamdaX_k, tt);
            neur_bezier[i].lamdaX(1)=neur_bezier[i].lamdaX_conver[1].linearConvert( neur_bezier[i].lamdaX(1), -20 * lamdaX_k, tt);
            neur_bezier[i].lamdaX(2)=neur_bezier[i].lamdaX_conver[2].linearConvert( neur_bezier[i].lamdaX(2), 10 * lamdaX_k, tt);
            neur_bezier[i].lamdaX(3)=neur_bezier[i].lamdaX_conver[3].linearConvert( neur_bezier[i].lamdaX(3), 10 * lamdaX_k, tt);
            neur_bezier[i].lamdaX(4)=neur_bezier[i].lamdaX_conver[4].linearConvert( neur_bezier[i].lamdaX(4), 10 * lamdaX_k, tt);
            neur_bezier[i].lamdaX(5)=neur_bezier[i].lamdaX_conver[5].linearConvert( neur_bezier[i].lamdaX(5), 10 * lamdaX_k, tt);
            neur_bezier[i].lamdaX(6)=neur_bezier[i].lamdaX_conver[6].linearConvert( neur_bezier[i].lamdaX(6), 10 * lamdaX_k, tt);
            neur_bezier[i].lamdaX(7)=neur_bezier[i].lamdaX_conver[7].linearConvert( neur_bezier[i].lamdaX(7), 10 * lamdaX_k, tt);
            neur_bezier[i].lamdaX(8)=neur_bezier[i].lamdaX_conver[8].linearConvert( neur_bezier[i].lamdaX(8), 40 * lamdaX_k, tt);

            neur_bezier[i].lamdaY(0)=neur_bezier[i].lamdaY_conver[0].linearConvert( neur_bezier[i].lamdaY(0), 10 * lamdaY_k, tt);
            neur_bezier[i].lamdaY(1)=neur_bezier[i].lamdaY_conver[1].linearConvert( neur_bezier[i].lamdaY(1), 10 * lamdaY_k, tt);
            neur_bezier[i].lamdaY(2)=neur_bezier[i].lamdaY_conver[2].linearConvert( neur_bezier[i].lamdaY(2), 1 * lamdaY_k, tt);
            neur_bezier[i].lamdaY(3)=neur_bezier[i].lamdaY_conver[3].linearConvert( neur_bezier[i].lamdaY(3), 30 * lamdaY_k, tt);
            neur_bezier[i].lamdaY(4)=neur_bezier[i].lamdaY_conver[4].linearConvert( neur_bezier[i].lamdaY(4), 30 * lamdaY_k, tt);
            neur_bezier[i].lamdaY(5)=neur_bezier[i].lamdaY_conver[5].linearConvert( neur_bezier[i].lamdaY(5), 10 * lamdaY_k, tt);
            neur_bezier[i].lamdaY(6)=neur_bezier[i].lamdaY_conver[6].linearConvert( neur_bezier[i].lamdaY(6), 60 * lamdaY_k, tt);
            neur_bezier[i].lamdaY(7)=neur_bezier[i].lamdaY_conver[7].linearConvert( neur_bezier[i].lamdaY(7), 30 * lamdaY_k, tt);
            neur_bezier[i].lamdaY(8)=neur_bezier[i].lamdaY_conver[8].linearConvert( neur_bezier[i].lamdaY(8), 10 * lamdaY_k, tt);

            // printf("\n swing_traj_switch_flag(%d) in i ni ni ni ni \n\n",i);            
            if( neur_bezier[i].lamdaX_conver[0].retConvDoneFlag()==true )
                swing_traj_switch_flag[i]=0;
        }

        Eigen::Vector3d temp_x;
        temp_x=leg_root.foot_swing_traj.block<3,1>(0,i);
        if( cpg_touch_down_scheduler(i)==0 &&  lift_cpg_phase_last(i)==1 ) // 如果当前是支撑态　且上一时刻是摆动态
        {
            lift_x_traj_rec(i)=temp_x(0);  // 记录进入支撑态那一时刻，ｘ方向的轨迹
        }
        lift_cpg_phase_last(i)=cpg_touch_down_scheduler(i);

        if( fabs( lift_x_traj_rec(i)- temp_x(0) ) >=0.75 *0.01 ) //如果在支撑态下，轨迹前进了　0.75 厘米
        {
            if( cpg_touch_down_scheduler(i)==0 && swing_traj_switch_flag[i]==0) //如果cpg是触地态
            {
                int tt=int(1/set_cpg_ctrl_cycle)*0.5*0.25;
                neur_bezier[i].lamdaX(0)=neur_bezier[i].lamdaX_conver[0].linearConvert( neur_bezier[i].lamdaX(0), 0, tt);
                neur_bezier[i].lamdaX(1)=neur_bezier[i].lamdaX_conver[1].linearConvert( neur_bezier[i].lamdaX(1), 0, tt);
                neur_bezier[i].lamdaX(2)=neur_bezier[i].lamdaX_conver[2].linearConvert( neur_bezier[i].lamdaX(2), 0, tt);
                neur_bezier[i].lamdaX(3)=neur_bezier[i].lamdaX_conver[3].linearConvert( neur_bezier[i].lamdaX(3), 0, tt);
                neur_bezier[i].lamdaX(4)=neur_bezier[i].lamdaX_conver[4].linearConvert( neur_bezier[i].lamdaX(4), 0, tt);
                neur_bezier[i].lamdaX(5)=neur_bezier[i].lamdaX_conver[5].linearConvert( neur_bezier[i].lamdaX(5), 0, tt);
                neur_bezier[i].lamdaX(6)=neur_bezier[i].lamdaX_conver[6].linearConvert( neur_bezier[i].lamdaX(6), 0, tt);
                neur_bezier[i].lamdaX(7)=neur_bezier[i].lamdaX_conver[7].linearConvert( neur_bezier[i].lamdaX(7), 0, tt);
                neur_bezier[i].lamdaX(8)=neur_bezier[i].lamdaX_conver[8].linearConvert( neur_bezier[i].lamdaX(8), 0, tt);

                neur_bezier[i].lamdaY(0)=neur_bezier[i].lamdaY_conver[0].linearConvert( neur_bezier[i].lamdaY(0), 0, tt);
                neur_bezier[i].lamdaY(1)=neur_bezier[i].lamdaY_conver[1].linearConvert( neur_bezier[i].lamdaY(1), 0, tt);
                neur_bezier[i].lamdaY(2)=neur_bezier[i].lamdaY_conver[2].linearConvert( neur_bezier[i].lamdaY(2), 0, tt);
                neur_bezier[i].lamdaY(3)=neur_bezier[i].lamdaY_conver[3].linearConvert( neur_bezier[i].lamdaY(3), 0, tt);
                neur_bezier[i].lamdaY(4)=neur_bezier[i].lamdaY_conver[4].linearConvert( neur_bezier[i].lamdaY(4), 0, tt);
                neur_bezier[i].lamdaY(5)=neur_bezier[i].lamdaY_conver[5].linearConvert( neur_bezier[i].lamdaY(5), 0, tt);
                neur_bezier[i].lamdaY(6)=neur_bezier[i].lamdaY_conver[6].linearConvert( neur_bezier[i].lamdaY(6), 0, tt);
                neur_bezier[i].lamdaY(7)=neur_bezier[i].lamdaY_conver[7].linearConvert( neur_bezier[i].lamdaY(7), 0, tt);
                neur_bezier[i].lamdaY(8)=neur_bezier[i].lamdaY_conver[8].linearConvert( neur_bezier[i].lamdaY(8), 0, tt);
            }
        }

        std::cout<<" neur_bezier[i].lamdaX"<<std::endl;
        std::cout<< neur_bezier[i].lamdaX<<std::endl;

        neur_bezier_lift_curve[i].Set_PX<< 0 , -1 , -3 , -5 , -7 , -5 , -3 , -1 , -0.5;  //输入单位cm 设置态升轨迹
        neur_bezier_lift_curve[i].Set_PY<< -3 + 3,-2.5 + 3, -2.3 + 3, -0.5 + 3, 1 + 3, 2.7 + 3, 3 + 3, 4.0 + 3, 3.75 + 3;  //输入单位cm　设置态升轨迹
        neur_bezier_lift_curve[i].Set_PX=neur_bezier_lift_curve[i].Set_PX*0.01  *0.15;   // *0.15 -> 轨迹缩小10倍,　
        neur_bezier_lift_curve[i].Set_PY=neur_bezier_lift_curve[i].Set_PY*0.01  *0.05;  // *0.05-> 大概抬高0.4cm
        if(swing_touch_leg_number(i)==1  && lift_stage_switching_flag==0) // 若　lift_stage_switching_flag==1　表示轨迹正在复原，此时不可调整轨迹
        {  
            _Cpg.cpg_stop_flag=1;

            double ttt;
            ttt=-1.0+_SimpleScheduler[i].retSimpleScheduler(1, 0.05)* 2.0;

            leg_root.foot_cross_traj.block<3,1>(0,i)= leg_root.foot_lift_traj.block<3,1>(0,i)+
                                neur_bezier_lift_curve[i].bezierCurve( ttt , 1);

            if( ttt>=0.9 )  // 因为大于0.9后，轨迹会返回-nan,也不知道为什么
            {   
                leg_root.foot_lift_traj.block<3,1>(0,i)=leg_root.foot_cross_traj.block<3,1>(0,i);  // 这个变量用于累加腿的抬升高度
                leg_root.foot_cross_object_est(i)=leg_root.foot_lift_traj(2,i);  //　记录腿的抬升高度并且用这个高度来估计腿遇到的障碍物．
                
                _SimpleScheduler[i].reSet();
                swing_touch_leg_number(i)=0;
                _Cpg.cpg_stop_flag=0;
            }
        }
    }

    //----------以下是 腿抬高轨迹foot_cross_traj的重置--------------//
    // 如果所有腿抬高都超过了2cm,那么所有腿可以重置
    if( fabs( leg_root.foot_cross_traj(2,0) )>1*0.01 &&  fabs( leg_root.foot_cross_traj(2,1) )>1*0.01 && fabs( leg_root.foot_cross_traj(2,2) )>1*0.01 &&
        fabs( leg_root.foot_cross_traj(2,3) )>1*0.01 &&  fabs( leg_root.foot_cross_traj(2,4) )>1*0.01 && fabs( leg_root.foot_cross_traj(2,5) )>1*0.01 && 
        lift_stage_flag==0 )
    {
        lift_stage_count++;
        if(lift_stage_count>= int(1/set_cpg_ctrl_cycle)*2  ) //如果在运行超过２个周期
        {
            lift_stage_flag=1; // 那么 down_stage_flag=1 即考虑让所有腿的　foot_dowmward_traj　都变回0;
            lift_stage_count=0;
        }
        // printf("lift_stage_count:%d\n",lift_stage_count);
    }

    if(lift_stage_flag==1 || lift_adaptiv_init_flag==1 ) // 让所有腿的　foot_cross_traj　都变回0
    {   
        Eigen::Vector3d Zero;
        Zero.setZero();
        // Eigen::Matrix<double, 1, 6> ZZero;
        // ZZero.setZero();

        for(int i=0; i<6; i++)
        {
            lift_stage_switching_flag=1;

            int tt=int(1/set_cpg_ctrl_cycle)*1;
            leg_root.foot_cross_traj.block<3,1>(0,i)=
                        foot_cross_traj_res[i].linearConvert(leg_root.foot_cross_traj.block<3,1>(0,i),Zero, tt);

            leg_root.foot_lift_traj.block<3,1>(0,i)=
                        foot_lift_traj_res[i].linearConvert(leg_root.foot_lift_traj.block<3,1>(0,i), Zero, tt);

            leg_root.foot_cross_object_est(i)=foot_cross_object_est_res[i].linearConvert(leg_root.foot_cross_object_est(i), 0, tt);

            if( foot_cross_traj_res[0].retConvDoneFlag()==true && foot_cross_traj_res[1].retConvDoneFlag()==true &&
            foot_cross_traj_res[2].retConvDoneFlag()==true && foot_cross_traj_res[3].retConvDoneFlag()==true &&
            foot_cross_traj_res[4].retConvDoneFlag()==true && foot_cross_traj_res[5].retConvDoneFlag()==true )
            {
                lift_stage_switching_flag=0;
                lift_stage_flag=0;
                printf("  done! \n");

                lift_adaptiv_init_flag=0;
            }
        }
    }
}

Eigen::Matrix<double,1,6>  dowm_cpg_phase_last;
Eigen::Matrix<double,1,6>  dowm_x_traj_rec;
void Hexapod::dowmwardReaction(void)
{
    // std::cout<<"ContactSimple.leg_suportingphase_contact_est"<<std::endl;
    // std::cout<<ContactSimple.leg_suportingphase_contact_est<<std::endl;

    // std::cout<<"leg_root.foot_swing_traj*100"<<std::endl;
    // std::cout<<leg_root.foot_swing_traj*100<<std::endl;

    // std::cout<<"dowm_x_traj_rec*100"<<std::endl;
    // std::cout<<dowm_x_traj_rec*100<<std::endl;

    // std::cout<<"--- next ---"<<std::endl;

    //----  以下是下探反应 ----//
    for (int i = 0; i < 6; i++)
    {
        Eigen::Vector3d temp_x;
        temp_x=leg_root.foot_swing_traj.block<3,1>(0,i);
        if( cpg_touch_down_scheduler(i)==0 &&  dowm_cpg_phase_last(i)==1 ) // 如果当前是支撑态　且上一时刻是摆动态
        {
            dowm_x_traj_rec(i)=temp_x(0);  // 记录进入支撑态那一时刻，ｘ方向的轨迹
        }
        dowm_cpg_phase_last(i)=cpg_touch_down_scheduler(i);

        if( ContactSimple.leg_suportingphase_contact_est(i)==0 && cpg_touch_down_scheduler(i)==0 ) //如果cpg支撑态 && leg_suportingphase_contact_est不是支撑态 
        {
            if( fabs( dowm_x_traj_rec(i)- temp_x(0) ) >=0.75 *0.01 && fabs( dowm_x_traj_rec(i))>=1 *0.01 ) //如果在支撑态下，轨迹前进了　0.75 厘米 && 记录的轨迹必需大于１cm
            {
                suporting_slip_leg_number(i)=1;
                printf("leg(%d) touch dowm silp!!\n ",i);
                // exit(0);
            }
        }

        if(suporting_slip_leg_number(i)==1 && down_stage_switching_flag==0) 
        {   
            _Cpg.cpg_stop_flag=1;
            leg_root.foot_dowmward_traj.block<3,1>(0,i)= leg_root.foot_dowm_traj.block<3,1>(0,i)+
                                    _LagrangeInterpolator[i].liftLegOne_cm(0.00001, -0.25, _SimpleScheduler[i].retSimpleScheduler(1, 0.1) );

            if(_SimpleScheduler[i].retTime()==1)
            {   
                leg_root.foot_dowm_traj.block<3,1>(0,i)=leg_root.foot_dowmward_traj.block<3,1>(0,i);  
                leg_root.foot_ditch_deepth_est(i)=leg_root.foot_dowm_traj(2,i);  
                
                _SimpleScheduler[i].reSet();
                suporting_slip_leg_number(i)=0;
                _Cpg.cpg_stop_flag=0;
            }
        }
    }

    //----------以下是 foot_dowmward_traj 的重置--------------//
    //  单腿恢复:
    // for( int i=0; i<6; i++ )
    // {
    //     if( cpg_touch_down_scheduler(i)==1 ) // 如果处于摆动态
    //     {
    //         Eigen::Vector3d Zero;
    //         Zero.setZero();
    //         int tt=int(1/set_cpg_ctrl_cycle)*1;
    //         leg_root.foot_dowmward_traj.block<3,1>(0,i)=
    //                     foot_dowmward_traj_res[i].linearConvert(leg_root.foot_dowmward_traj.block<3,1>(0,i),Zero, tt);

    //         leg_root.foot_dowm_traj.block<3,1>(0,i)=
    //                     foot_dowm_traj_res[i].linearConvert(leg_root.foot_dowm_traj.block<3,1>(0,i), Zero, tt);

    //         leg_root.foot_ditch_deepth_est(i)=foot_ditch_deepth_est_res[i].linearConvert(leg_root.foot_ditch_deepth_est(i), 0, tt);
    //     }
    // }

    //  所有腿恢复:  如果所有腿降都超过了2cm
    if( fabs( leg_root.foot_dowmward_traj(2,0) )>2*0.01 &&  fabs( leg_root.foot_dowmward_traj(2,1) )>2*0.01 && fabs( leg_root.foot_dowmward_traj(2,2) )>2*0.01 &&
        fabs( leg_root.foot_dowmward_traj(2,3) )>2*0.01 &&  fabs( leg_root.foot_dowmward_traj(2,4) )>2*0.01 && fabs( leg_root.foot_dowmward_traj(2,5) )>2*0.01 &&
        down_stage_flag==0 )
    {
        down_stage_count++;
        if(down_stage_count>= int(1/set_cpg_ctrl_cycle)*1  ) //如果在运行超过1个周期
        {
            down_stage_flag=1; // 那么 down_stage_flag=1 即考虑让所有腿的　foot_dowmward_traj　都变回0;
            down_stage_count=0;
        }
    }

    if(down_stage_flag==1 || dowm_adaptiv_init_flag==1 ) // 让所有腿的　foot_dowmward_traj　都变回0
    {   
        Eigen::Vector3d Zero;
        Zero.setZero();
        for(int i=0; i<6; i++)
        {
            down_stage_switching_flag=1;
            int tt=int(1/set_cpg_ctrl_cycle)*1;
            leg_root.foot_dowmward_traj.block<3,1>(0,i)=
                        foot_dowmward_traj_res[i].linearConvert(leg_root.foot_dowmward_traj.block<3,1>(0,i),Zero, tt);

            leg_root.foot_dowm_traj.block<3,1>(0,i)=
                        foot_dowm_traj_res[i].linearConvert(leg_root.foot_dowm_traj.block<3,1>(0,i), Zero, tt);

            leg_root.foot_ditch_deepth_est(i)=foot_ditch_deepth_est_res[i].linearConvert(leg_root.foot_ditch_deepth_est(i), 0, tt);

            // std::cout<<"leg_root.foot_dowmward_traj*100"<<std::endl;
            // std::cout<<leg_root.foot_dowmward_traj*100<<std::endl;

            if( foot_dowmward_traj_res[0].retConvDoneFlag()==true && foot_dowmward_traj_res[1].retConvDoneFlag()==true &&
            foot_dowmward_traj_res[2].retConvDoneFlag()==true && foot_dowmward_traj_res[3].retConvDoneFlag()==true &&
            foot_dowmward_traj_res[4].retConvDoneFlag()==true && foot_dowmward_traj_res[5].retConvDoneFlag()==true )
            {
                down_stage_switching_flag=0;
                down_stage_flag=0;
                printf("  done! \n");

                dowm_adaptiv_init_flag=0;
            }
        }
    }
}

void Hexapod::adaptive_control(void)
{
    printf("\n ----------liftReaction---------- \n");
    liftReaction();
    std::cout<<"leg_root.foot_cross_traj*100"<<std::endl;
    for (int i = 0; i < 6; i++)
    {
        printf(" %f ",leg_root.foot_cross_traj(2,i)*100);
    }
    printf(" \n");

    printf("\n ----------dowmwardReaction---------- \n");
    dowmwardReaction();
    std::cout<<"leg_root.foot_dowmward_traj*100"<<std::endl;
    for (int i = 0; i < 6; i++)
    {
        printf(" %f ",leg_root.foot_dowmward_traj(2,i)*100);
    }
    printf(" \n");

    for (int i = 0; i < 6; i++)
    {
        leg_root.foot_trajectory.block<3,1>(0,i)=leg_root.foot_cross_traj.block<3,1>(0,i)+leg_root.foot_dowmward_traj.block<3,1>(0,i)+leg_root.foot_swing_traj.block<3,1>(0,i); 
    }

    Eigen::Matrix<double, 1, 6> temp_hight, temp_deepth;
    temp_hight=foot_cross_hight_sort.sort(leg_root.foot_cross_object_est);  //lcc 20230519:通过冒泡排序，将跨越高度提出来，将来用做机身高度调节
    temp_deepth=foot_ditch_deepth_sort.sort(leg_root.foot_ditch_deepth_est);  

    set_z_deviation=deviation_conver[2].linearConvert(set_z_deviation, temp_hight(5)/1.5, 10);
}