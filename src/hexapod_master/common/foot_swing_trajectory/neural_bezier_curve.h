#ifndef _NEURALBEZIERCURVE_H
#define _NEURALBEZIERCURVE_H

#include "../utilities/linear_trans.h"
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <stdio.h>
using namespace Eigen;

double * FourthPlane(double CPGXout,int CPGYout);

class neural_bezier_curve
{
    private:
        double delaT=0.005;
        double N=1/delaT; //%  200段，201个点
        double t=0;
        double Postion[3]={0};
        //%%%%%%%%% X 方向
        int n=9;   //%% 控制点数 n  
        int layerNum=n-1;   //% n层神经网络能够控制n个位置点，n-1段曲线，所以有n-1个可控制参数

        // double  Set_PX[9]={-3 , -2.5 , -1.5 , -0.5 , 0 , 0.5 , 1.5 , 2.5 , 3 };  //%%初始的轨迹图数据1
        // double  Set_PY[9]={0*2 , 1.5*2 , 2.5*2 , 3.5*2 , 4.5*2 , 3.5*2 , 2.5*2 , 1.5*2 , 0*2 };

        double  Set_PX[9]={-3 ,-2.5, -2.3, -0.5, 1, 2.7, 3, 4.0, 3};  //%%初始的轨迹图数据1
        double  Set_PY[9]={0 , 3 , 5 , 7 , 9 , 7 , 5 , 3 , 0 };
        //   PX=[-3 ,-2.5, -2.3, -0.5, 1, 2.7, 3, 4.0, 3];  %%初始的轨迹图数据1
        //   PY=[0 , 3 , 5 , 7 , 9 , 7 , 5 , 3 , 0 ];

        // double  Set_PX[9]={-3.75 ,-2.5, -2.3, 0, 1.25, 2.7, 3.75, 5.0, 3.75};  //论文轨迹
        // double  Set_PY[9]={0,4,8,7.5,4,8,6,2,0};

        double  PX[9]={0};
        double  PY[9]={0};
        
        int TnIndex=0;

        double WXout=0,wx=0,WYout=0,wy=0;

        MatrixXd XCOM_BM,YCOM_BM;
        // %%%%%%%%%% X 方向
        MatrixXd Xoutput,Xinput,Xoutput0,Youtput,Yinput,Youtput0;
        MatrixXd kX,kY,aX,aY,XCn,YCn;
        // MatrixXd lamdaX,lamdaY;
        // Eigen::Matrix<double,1,6> lamdaX,lamdaY;

        double Xinput_[8]={0},Xinput_last[8]={0};  // layerNum=n-1=8
        double Xoutput_[8]={0},Xoutput_last[8]={0};
        double Xoutput0_=0,Xoutput0_last[8]={0};
        double Yinput_[8]={0},Yinput_last[8]={0};
        double Youtput_[8]={0},Youtput_last[8]={0};
        double Youtput0_=0,Youtput0_last[8]={0};
        double XCOM_BM_=0,XCOM_BM_last[8]={0};
        double YCOM_BM_=0,YCOM_BM_last[8]={0};

        double comb(double n,double m);
    public:
        MatrixXd lamdaX,lamdaY;
        double height_k=1.3,length_k=1;
        double height_k_last=0,length_k_last=0;

        linear_trans lamdaX_conver,lamdaY_conver;

        Eigen::Vector3d  bezierCurve(double _Cpg,double phase);
        
        void setCurveHight(double h_k)
        {
            if(height_k_last!=height_k)
            {
                 
                for(int i=0;i<n;i++)
                {
                    PY[i]=Set_PY[i]*height_k;
                    // printf("PY:%f ",PY[i]);
                }
                height_k_last=height_k;  
                // printf("h_k:%f \n",height_k);
            }

        }
        void setCurveLength(double l_k)
        {
            if(length_k_last!=length_k)
            {
                for(int i=0;i<n;i++)
                    PX[i]=Set_PX[i]*length_k;   
                length_k_last=length_k;    
            }
        }
        
        neural_bezier_curve()
        {

            for(int i=0;i<n;i++)
            {
                Set_PX[i]=Set_PX[i]*0.01;
                Set_PY[i]=Set_PY[i]*0.01;
            }

            kX.resize(1,n);
            kY.resize(1,n);
            aX.resize(1,n);
            aY.resize(1,n);
            XCn.resize(1,n);
            YCn.resize(1,n);

            Xinput.resize(layerNum,N+1);
            Xoutput.resize(layerNum,N+1);
            Xoutput0.resize(1,N+1);

            Yinput.resize(layerNum,N+1);
            Youtput.resize(layerNum,N+1);
            Youtput0.resize(1,N+1);
            
            XCOM_BM.resize(1,N+1);
            YCOM_BM.resize(1,N+1);

            lamdaX.resize(1,n);
            lamdaY.resize(1,n);
        }

        double retDesStepLength(void)
        {   
            double step_length;
            step_length=PX[n-1]-PX[0];
            return step_length;
        }

        double retDesStepHight(void)
        {   
            double step_hight;
            step_hight=PY[n-1]-PY[0];
            // printf("PY[n-1]:%f \n",PY[n-1]);
            return step_hight;
        }

};


class YQRDL
{
    public:

};

#endif
