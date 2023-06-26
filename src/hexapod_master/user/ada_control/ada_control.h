#ifndef _ADACONTROL_H
#define _ADACONTROL_H

#include<stdio.h>
#include<math.h>
#include<Eigen/Core>
using namespace Eigen;

// zyt_control 2023.4.19
class ada_ctrl
{
    private:

        Matrix<double,3,1>_errp,_errv,_Ft,_err,_taufb,_tau,_tauff;
        Matrix<double,3,3>_Kt,_Dt;
        double _gama,_beta,_a,_b;
        double _toque[3]={0};
        double _kp[3]={0},_kd[3]={0};
        double lim=30;
        
    public:
        void setAda(double beta,double a,double b)
        {
            _beta=beta;  _a=a;  _b=b;
        }
        void setKp(double kp1,double kp2,double kp3)
        {
            _kp[0]=kp1;  _kp[1]=kp2;  _kp[2]=kp3;
        }
        void setKd(double kd1,double kd2,double kd3)
        {
            _kd[0]=kd1;  _kd[1]=kd2;  _kd[2]=kd3;
        }

        double*  adaTau(double * joi_act_pos,double * joi_des_pos,double * joi_act_vel,double * joi_des_vel)
        {
            for (int i = 0; i < 3; i++)
            {   
                _errp(i)=joi_des_pos[i]-joi_act_pos[i];
                _errv(i)=0-joi_act_vel[i];
                _err(i)=_errp(i)+_beta*_errv(i);
                _tauff[i]=_kp[i]*(joi_des_pos[i]-joi_act_pos[i])+_kd[i]*(0-joi_act_vel[i]);
            }   
            _gama=_a/(1+_b*(pow(_err.norm(),2)));
            _Ft=_err/_gama;
            _Kt=_Ft*_errp.transpose();
            _Dt=_Ft*_errv.transpose();
            _taufb=_Ft+_Kt*_errp+_Dt*_errv;
            _tau=_tauff+_taufb;
            for (int i = 0; i < 3; i++)
            {
                _toque[i]=_tau[i];
                if (_toque[i]>=lim){_toque[i]=lim;}
                if(_toque[i]<=-lim){_toque[i]=-lim;}
            }
            return _toque;
        }
      
        // double*  iteTau(double * joi_act_pos,double * joi_des_pos,double * joi_act_vel,double * joi_des_vel)
        // {

        // }

    


};



#endif
