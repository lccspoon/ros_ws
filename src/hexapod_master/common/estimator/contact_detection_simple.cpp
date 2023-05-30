#include "contact_detection_simple.h"


Eigen::Matrix<double,1,6> contact_detection_simple::swingphase_contact_est(Eigen::Matrix<double,3,6> footend_force,
                                                        Eigen::Matrix<double,1,6> plan_touch_down_scheduler,
                                                        Eigen::Matrix<double,3,6> foot_swing_traj)
{

    for (int i=0; i<6; ++i) 
    {
        Eigen::Vector3d foot_for = footend_force.block<3,1>(0,i);    

        //法一：
        // Eigen::Vector3d foot_contacts_estimated;
        // foot_contacts_estimated.setZero();

        // // 每一条腿的足端力分为三个方向，　每一条腿的触碰检测也分为３个方向; 如果力大于１，则该方向foot_contacts_estimated＝０即表示触碰
        // if( fabs(foot_for(0)) >= 1 ) foot_contacts_estimated(0)=0;  else foot_contacts_estimated(0)=1;
        // if( fabs(foot_for(1)) >= 1 ) foot_contacts_estimated(1)=0;  else foot_contacts_estimated(1)=1;
        // if( fabs(foot_for(2)) >= 1 ) foot_contacts_estimated(2)=0;  else foot_contacts_estimated(2)=1;
        
        // contact_estimate.block<3,1>(0,i)=foot_contacts_estimated;  

        //法二：　直接通过合力来判断
        double f_sum;
        f_sum=  sqrt( foot_for(0)*foot_for(0)+foot_for(1)*foot_for(1)+foot_for(2)*foot_for(2) );
        if(f_sum>=2)
            contact_estimate_schedual(i)=0;   // 0 :即合理大于２，用０表示触碰
        else
            contact_estimate_schedual(i)=1;
    }

    //lcc 摆动触碰估计
    leg_swingphase_contact_est.setZero();
    for(int i=0;i<6;i++)
    {       
            Eigen::Vector3d temp;
            temp=foot_swing_traj.block<3,1>(0,i);
            // printf("foot_swing_traj (2):%f  ",temp(2));

            //如果在cpg_touch_down_scheduler=1中认为腿摆动, 且contact_estimate_schedual=0即认为腿触碰，　
            //                                          且摆动轨迹的高度大于３cm　－＞  则认为摆动触碰，令leg_swingphase_contact_est＝１;
            if(plan_touch_down_scheduler(i)==1 && contact_estimate_schedual(i)==0 && temp(2)>=3*0.01)
            {       
                    // printf("\n");
                    // printf("cpg_touch_down_scheduler(%d):%f contact_est_scheduler(%d):%f temp(2):%f \n",
                    // i, plan_touch_down_scheduler(i), i, contact_estimate_schedual(i), temp(2));
                    // printf(" Touch object, robot stop and program exit! \n");
                    leg_swingphase_contact_est(i)=1;   //表示摆动时触碰东西
                    // exit(0);
            }
    }

    //lcc 20230519:触地估计
    for(int i=0;i<6;i++)
    {       
            if(plan_touch_down_scheduler(i)==0 && contact_estimate_schedual(i)==0)//如果contact_estimate_schedual=0即表示触碰物体　&&　cpg=0即表示支撑态
            {       
                leg_suportingphase_contact_est(i)=1;   //1表示触地
            } 
            else if(plan_touch_down_scheduler(i)==1)  //lcc 如果cpg处于摆动相, 只有到达摆动态，腿的触地信号才会重置
            {
                leg_suportingphase_contact_est(i)=0;   //0表示bu触地
            }
    }

    std::cout<<"footend_force"<<std::endl;
    std::cout<<footend_force<<std::endl;
    std::cout<<"plan_touch_down_scheduler"<<std::endl;
    std::cout<<plan_touch_down_scheduler<<std::endl;
    std::cout<<"contact_estimate_schedual"<<std::endl;
    std::cout<<contact_estimate_schedual<<std::endl;
    std::cout<<"leg_suportingphase_contact_est"<<std::endl;
    std::cout<<leg_suportingphase_contact_est<<std::endl;

    // std::cout<<"leg_swingphase_contact_est"<<std::endl;
    // std::cout<<leg_swingphase_contact_est<<std::endl;


    return leg_swingphase_contact_est;
}




