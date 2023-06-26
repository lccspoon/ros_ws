#ifndef _FUSELAGEATTITUDE_H
#define _FUSELAGEATTITUDE_H

#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <stdio.h>
#include "../utilities/linear_trans.h"

using namespace Eigen;
// double foo_and_bod_adj_and_map::RRR_YAW=0;
class FooTipAndBodAdjMap
{
    private:

        Matrix<double,3,3> _Rx,_Ry,_Rz;
        Matrix<double,3,1> end_effore,Body_P_,Frame_Body,end_effore_;
        Matrix<double,4,1> OB_,OB_temp_,OO_,OA_,deviation_,AB_,PositonBias_;
        Matrix<double,3,1>  _end_effore,Frame_,Frame_EndEffore_;
        Matrix<double,4,4> RRR;

        double RRR_YAW,RRR_ROLL,RRR_PITCH,X_DEVIATION,Y_DEVIATION,Z_DEVIATION;

        double Postion[3];
        double  traj[3];
    public:

        void setInitEndEfforeAndPBias(Eigen::Vector3d xyz_,double * bias_xyz,double WB_X,double WB_Y);
        Eigen::Vector3d trajAndAdjustMapping(Eigen::Vector3d traj);
        double * footTipPosRefToBody(void);
        Eigen::Vector3d legOriRefToBody(void);
        void msgShow(void);

        // static double RRR_YAW,RRR_ROLL,RRR_PITCH,X_DEVIATION,Y_DEVIATION,Z_DEVIATION;
        double rzz;

        double rrr_yaw=0,rrr_roll=0,rrr_pitch=0,x_deviation=0,y_deviation=0,z_deviation=0;
        linear_trans YawLinTran,RolLinTran,PitLinTran,XdeLinTran,YdeLinTran,ZdeLinTran,RzzLinTran;

        double step_amplitude=0;
        linear_trans SteAmpLinTran;

        void fuselageAttiuCtrl(double r_yaw,double r_roll,double r_pitch)
        {
            RRR_YAW=r_yaw;RRR_ROLL=r_roll;RRR_PITCH=r_pitch;

        }
        void fuselageDeviCtrl(double xx_deviate,double yy_deviate,double zz_zdeviate)
        {
            X_DEVIATION=xx_deviate;Y_DEVIATION=yy_deviate;Z_DEVIATION=zz_zdeviate;

        }

};

// class FuselageCtrl
// {
//     public:
//         Fuselage 

// };


#endif