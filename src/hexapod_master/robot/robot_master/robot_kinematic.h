#ifndef _KINEMATIC_H
#define _KINEMATIC_H
#include "robot_parameter.h"
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

class kinematic
{   private:
        double invAngle[3];
        double forwardPosition[3];
        Eigen::Matrix<double,3,3> Jacobian,Jacobian_T,Jacobian_T_inv;
    public:
        // kinematic(double L1,double L2,double L3)
        // {
        //     invAngle[1]=L1;
        //     invAngle[1]=L2;
        //     invAngle[1]=L3;
        //     // robot.LEG_DH_PARAM1=L1;
        //     // robot.LEG_DH_PARAM2=L2;
        //     // robot.LEG_DH_PARAM3=L3;
        // }
        static double LEG_DH_PARAM1,LEG_DH_PARAM2,LEG_DH_PARAM3;
        static void setDhParam(double L1,double L2,double L3)
        {
            LEG_DH_PARAM1=L1;
            LEG_DH_PARAM2=L2;
            LEG_DH_PARAM3=L3;

        }
        void showInfo(void);

        
        Eigen::Vector3d invKinematic(int leg_number, Eigen::Vector3d Position_);
        Eigen::Vector3d forwardKinematic(int leg_number, Eigen::Vector3d angle_);
        // Eigen::Matrix<double,3,3> jacobiTranInv(char **leg_name, double angle[0],double angle[1],double angle[2]);
        Eigen::Matrix<double,3,3> jacobiTranInv(int leg_number,Eigen::Vector3d angle_);
        Eigen::Matrix<double,3,3> jacobi(void);

};

#endif