#include "robot_parameter.h"
#include "robot_main.h"

double countttt=-1;
int get_size_flag=0;
void Hexapod::setStepSize()
{
        // //lcc 得到原始设计轨迹的步高步长，只运行一次
        if( get_size_flag==0 )
                for(int i=0; i<6; i++)
                {
                        Eigen::Vector3d get_step_size;
                        countttt=countttt+0.01;
                        if(countttt>=1)
                                get_size_flag=1;
                        get_step_size=neur_bezier[i].bezierCurve( countttt, 1.0);
                        leg_root.step_original_hight(i)=get_original_step_higtt_sort[i].sort_continuet( get_step_size(2) );

                        get_step_size=neur_bezier[i].bezierCurve( 0.99, 1.0);
                        leg_root.step_original_length(i)=get_original_step_length_sort[i].sort_continuet( get_step_size(0) ) * 2 ; // 步长关于0点左右对称,所以*2
        
                        // std::cout<<"des_step_l / leg_root.step_original_length(i)"<<std::endl;      
                        // std::cout<<des_step_l / leg_root.step_original_length(i)<<std::endl;
                        // std::cout<<"des_step_h / leg_root.step_original_hight(i)"<<std::endl;      
                        // std::cout<<des_step_h / leg_root.step_original_hight(i)<<std::endl;
                        // std::cout<<"leg_root.step_original_hight"<<std::endl;      
                        // std::cout<<leg_root.step_original_hight<<std::endl;
                        // std::cout<<"leg_root.step_original_length"<<std::endl;      
                        // std::cout<<leg_root.step_original_length<<std::endl;
                        // exit(0);
                }

        for(int i=0; i<6; i++)
        {
                // leg_root.step_des_hight(i)=get_des_step_hight_sort[i].sort_continuet(  leg_root.foot_swing_traj(2,i) );

                // 此处得到的是轨迹x方向的最大值，注意最大值不一定是步长，但差不多
                // leg_root.step_des_length(i)=get_des_step_length_sort[i].sort_continuet( leg_root.foot_swing_traj(0,i) ) * 2 ; // 步长关于0点左右对称,所以*2，


                // /2是因为设计的补偿值其实是直接覆给　摆动长度和支撑长度的．一个周期的补偿＝摆动长度＋支撑长度
                double des_step_l=leg_root.step_set_length(i)/2 , des_step_h=leg_root.step_set_hight(i) ;  
                set_step_length_k(i)= des_step_l / leg_root.step_original_length(i);
                set_step_hight_k(i)= des_step_h / leg_root.step_original_hight(i);
        }
        // std::cout<<"leg_root.step_original_hight"<<std::endl;      
        // std::cout<<leg_root.step_original_hight<<std::endl;
        // std::cout<<"leg_root.step_original_length"<<std::endl;      
        // std::cout<<leg_root.step_original_length<<std::endl;

        // std::cout<<"leg_root.step_set_hight"<<std::endl;      
        // std::cout<<leg_root.step_set_hight*100<<std::endl;
        // std::cout<<"leg_root.step_set_length"<<std::endl;      
        // std::cout<<leg_root.step_set_length*100<<std::endl;

        // std::cout<<"leg_root.step_des_hight"<<std::endl;      
        // std::cout<<leg_root.step_des_hight<<std::endl;
        // std::cout<<"leg_root.step_des_length"<<std::endl;      
        // std::cout<<leg_root.step_des_length<<std::endl;

        for(int i=0; i<6; i++)
        {
                neur_bezier[i].length_k=_step_length_k_conver[i].linearConvert(neur_bezier[i].length_k,set_step_length_k(i),60);
                neur_bezier[i].height_k=_step_hight_k_conver[i].linearConvert(neur_bezier[i].height_k,set_step_hight_k(i),60);   
        }
}

void Hexapod::parInit()
{
    if(set_para_init_flag==1)   
    {
        set_x_deviation=0; set_y_deviation=0; set_z_deviation=0; 
        set_yaw=0; set_roll=0; set_pitch=0;

        Eigen::Matrix<double,1,6> one;
        one.setOnes();
        leg_root.step_set_length=one* 7*0.01;  //lcc 步长7cm
        leg_root.step_set_hight=one*5  * 0.01;   //lcc　步高5cm

        set_para_init_flag=0;
        #if HARD_WARE==1
                set_cpg_ctrl_cycle=0.0025;
        #elif HARD_WARE==2
                // set_cpg_ctrl_cycle=0.0015;   //lcc 一个步态周期的点数＝１／set_cpg_ctrl_cycle
                set_cpg_ctrl_cycle=0.00075;   //lcc 一个步态周期的点数＝１／set_cpg_ctrl_cycle
        #endif
        printf("init\n");

        cpg_touch_down_scheduler<< 404, 404, 404, 404, 404, 404;
        cpg_switch_period=120;

        #if HARD_WARE==2  //lcc 20230329: 开启仿真
                world_root.body_des_pos=GazeboSim.retOdoPostion();
        #endif
    }
}

Eigen::Matrix<double, 3, 6> foot_des_pos_last;
Eigen::Matrix<double,1,6>  cpg_phase_last;
Eigen::Matrix<double,1,6>  phase_count;
Eigen::Matrix<double,1,6>  caculate_flag;
Eigen::Matrix<double, 3, 6> foot_des_vel;
Eigen::Matrix<double, 3, 1> body_des_vel;
int leg_number;
void Hexapod::getDesPosAndVel()
{
        int tt=int(1/set_cpg_ctrl_cycle);
        for(int i=0; i<6; i++)
        {

                if( cpg_touch_down_scheduler(i)==0 &&  cpg_phase_last(i)==1 ) // 如果当前是支撑态　且上一时刻是摆动态
                {       
                        caculate_flag(i)=1;  // 那么允许该腿i参与速度的计算
                }
                else if( cpg_touch_down_scheduler(i)==1 &&  cpg_phase_last(i)==1  )
                {
                        caculate_flag(i)=0;  
                }

                if( caculate_flag(i)==1 )
                {
                        phase_count(i)++;
                        if( phase_count(i)>=2 )
                        {
                                // 足端期望速度 = （本次足端期望位置 - 上次足端期望位置）/ 时间
                             foot_des_vel.block<3,1>(0,i) = -( leg_root.foot_des_pos.block<3,1>(0,i) - foot_des_pos_last.block<3,1>(0,i) ) / dt_s;
                        }
                        foot_des_pos_last.block<3,1>(0,i) = leg_root.foot_des_pos.block<3,1>(0,i);
                }
                else
                {       
                        phase_count(i)=0;
                        foot_des_vel.block<3,1>(0,i).setZero();
                        foot_des_pos_last.block<3,1>(0,i).setZero();
                }
                cpg_phase_last(i)=cpg_touch_down_scheduler(i);
        }

        for(int i=0; i<6;i++)
        {       
                if( foot_des_vel(0,i)!=0 )
                        leg_number++;  //得到参与速度计算的腿速度
        }
        if( leg_number!=0 ) 
                body_des_vel=   (foot_des_vel.block<3,1>(0,0)+ foot_des_vel.block<3,1>(0,1) +  foot_des_vel.block<3,1>(0,2) +     //求平均
                        foot_des_vel.block<3,1>(0,3) +  foot_des_vel.block<3,1>(0,4) +  foot_des_vel.block<3,1>(0,5) )/leg_number;
        else 
                body_des_vel.setZero();

        world_root.body_des_vel=body_des_vel; //得到期望速度
        world_root.body_des_pos= body_des_vel*dt_s + world_root.body_des_pos; //得到期望位置

        for(int i=0;i<6;i++)
                world_root.foot_des_pos.block<3,1>(0,i)  = body_root.foot_des_pos.block<3,1>(0,i) + world_root.body_des_pos;

        // std::cout<<"dt_s"<<std::endl;
        // std::cout<<dt_s<<std::endl;
        // std::cout<<"leg_number"<<std::endl;
        // std::cout<<leg_number<<std::endl;
        // std::cout<<"phase_count"<<std::endl;
        // std::cout<<phase_count<<std::endl;
        // std::cout<<"foot_des_pos_last"<<std::endl;
        // std::cout<<foot_des_pos_last<<std::endl;
        // std::cout<<"foot_des_vel"<<std::endl;
        // std::cout<<foot_des_vel<<std::endl;
        // std::cout<<"world_root.des_vel"<<std::endl;
        // std::cout<<world_root.des_vel<<std::endl;
        // std::cout<<"body_root.des_pos"<<std::endl;
        // std::cout<<world_root.des_pos<<std::endl;
        leg_number=0;
}
