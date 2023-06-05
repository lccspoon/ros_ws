#include "contact_detection_simple.h"


Eigen::Matrix<double,1,6> contact_detection_simple::contact_estimation(Eigen::Matrix<double,3,6> footend_force)
{

    // for (int asad = 0; asad < 3; asad++)
    // {
    //     printf("contact_detection_simple  asad:%d\n",asad);
    // }
    

    for (int i=0; i<6; ++i) 
    {
        Eigen::Vector3d foot_for = footend_force.block<3,1>(0,i);    

        Eigen::Vector3d foot_contacts_estimated;
        foot_contacts_estimated.setZero();
        if( fabs(foot_for(0)) >= 1 ) foot_contacts_estimated(0)=0;  else foot_contacts_estimated(0)=1;
        if( fabs(foot_for(1)) >= 1 ) foot_contacts_estimated(1)=0;  else foot_contacts_estimated(1)=1;
        if( fabs(foot_for(2)) >= 1 ) foot_contacts_estimated(2)=0;  else foot_contacts_estimated(2)=1;
        
        contact_estimate.block<3,1>(0,i)=foot_contacts_estimated;

        double f_sum;
        f_sum=  sqrt( foot_for(0)*foot_for(0)+foot_for(1)*foot_for(1)+foot_for(2)*foot_for(2) );
        if(f_sum>=2)
            contact_estimate_schedual(i)=0;
        else
            contact_estimate_schedual(i)=1;
        // printf("contact_detection_simple  iii:%d\n",i);
    }
    // std::cout<<"footend_force"<<std::endl;
    // std::cout<<footend_force<<std::endl;
    // std::cout<<"contact_estimate"<<std::endl;
    // std::cout<<contact_estimate<<std::endl;
    // std::cout<<"contact_estimate_schedual"<<std::endl;
    // std::cout<<contact_estimate_schedual<<std::endl;

    return contact_estimate_schedual;
}




