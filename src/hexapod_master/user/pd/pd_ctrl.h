#ifndef _PDCONTROL_H
#define _PDCONTROL_H

#include <stdio.h>
#include<math.h>
#include<Eigen/Core>
using namespace Eigen;

class pd_ctrl
{
    private:
        // double _tau[3]={0};
        double _position_last[3]={0},_velocity_last[3]={0};
        double _kp[3]={0},_kd[3]={0},_ki[3]={0};
        double err[3]={0},err_last[3]={0},integral[3]={0};
        double increment_val[3]={0},err_next[3]={0};
        int _enable[3]={0};
        double _RAD=180/3.1415926;
        Matrix<double,3,1>_tau;
        
    public:
        void setKp(double kp1,double kp2,double kp3)
        {
            _kp[0]=kp1;  _kp[1]=kp2;  _kp[2]=kp3;
        }
        void setKd(double kd1,double kd2,double kd3)
        {
            _kd[0]=kd1;  _kd[1]=kd2;  _kd[2]=kd3;
        }
        void setKi(double kd1,double kd2,double kd3)
        {
            _ki[0]=kd1;  _ki[1]=kd2;  _ki[2]=kd3;
        }

        Eigen::Matrix<double,3,1>  pdTau(Matrix<double,3,1>joi_act_pos,Matrix<double,3,1>joi_des_pos,Matrix<double,3,1>joi_act_vel,Matrix<double,3,1> joi_des_vel)
        {
            for (int i = 0; i < 3; i++)
            {   
                _tau[i]=_kp[i]*(joi_des_pos[i]-joi_act_pos[i])+_kd[i]*(0-joi_act_vel[i]);


            }
            return _tau;
        }

        Eigen::Matrix<double,3,1> positonPidTau(Matrix<double,3,1>joi_act_pos,Matrix<double,3,1>joi_des_pos,Matrix<double,3,1>joi_act_vel,Matrix<double,3,1> joi_des_vel)
        {
            for (int i = 0; i < 3; i++)
            {   
                err[i]=joi_des_pos[i]-joi_act_pos[i];

                integral[i]+=err[i];

                _tau[i]= _kp[i]*err[i] +
                         _kd[i]*(err[i]-err_last[i]) +
                         _ki[i]*integral[i];

                err_last[i]=err[i];
            }
            return _tau;
        }

        Eigen::Matrix<double,3,1>  incrementPidTau(Matrix<double,3,1>joi_act_pos,Matrix<double,3,1>joi_des_pos,Matrix<double,3,1>joi_act_vel,Matrix<double,3,1> joi_des_vel)
        {
            for (int i = 0; i < 3; i++)
            {   


                err[i]=joi_des_pos[i]-joi_act_pos[i];


                increment_val[i]=   _kp[i]*(err[i]-err_next[i]) +
                                    _kd[i]*(err[i]-2*err_next[i]+err_last[i]) +
                                    _ki[i]*err[i];

                _tau[i]+=increment_val[i];

                err_last[i]=err_next[i];
                err_next[i]=err[i];
            }
            return _tau;
        }

};





#endif
