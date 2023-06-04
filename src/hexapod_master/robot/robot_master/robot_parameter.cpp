#include "robot_parameter.h"
#include "robot_main.h"

double countttt=-1;
int get_size_flag=0;
void Hexapod::setStepSize()
{
        // //lcc 得到轨迹的步高步长
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

                double des_step_l=leg_root.step_set_length(i) * 0.01, des_step_h=leg_root.step_set_hight(i) * 0.01;  
                set_step_length_k(i)= des_step_l / leg_root.step_original_length(i);
                set_step_hight_k(i)= des_step_h / leg_root.step_original_hight(i);
        }
        // std::cout<<"leg_root.step_original_hight"<<std::endl;      
        // std::cout<<leg_root.step_original_hight<<std::endl;
        // std::cout<<"leg_root.step_original_length"<<std::endl;      
        // std::cout<<leg_root.step_original_length<<std::endl;

        // std::cout<<"leg_root.step_set_hight"<<std::endl;      
        // std::cout<<leg_root.step_set_hight<<std::endl;
        // std::cout<<"leg_root.step_set_length"<<std::endl;      
        // std::cout<<leg_root.step_set_length<<std::endl;


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
            leg_root.step_set_length=one*3;  //lcc 步长３cm
            leg_root.step_set_hight=one*4;   //lcc　步高４cm

            set_para_init_flag=0;
            #if HARD_WARE==1
                    set_cpg_ctrl_cycle=0.0025;
            #elif HARD_WARE==2
                    set_cpg_ctrl_cycle=0.0015;   //lcc 一个步态周期的点数＝１／set_cpg_ctrl_cycle
            #endif
            printf("init\n");

            cpg_touch_down_scheduler<< 404, 404, 404, 404, 404, 404;
    }
}