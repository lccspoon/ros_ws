#ifndef XZBLEGODOMERTER_H
#define XZBLEGODOMERTER_H
/* 
 *Last editing: 2022/5/22
 *xzb
 */

#include<iostream>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

#define num_leg  6


// using namespace Eigen;

struct BodyStateData_LegOdom{
    Eigen::Matrix<double,3,1>_pCOM;
    Eigen::Matrix<double,3,1>_vCOM;
    Eigen::Matrix<double,3,1>_rpyCOM;
    Eigen::Matrix<double,3,1>_aCOM;//从传感器中获得的
    Eigen::Matrix<double,3,1>_rpydotCOM;//从传感器中获得的
    Eigen::Matrix<double,3*num_leg,1> _GRF;
    // Eigen::Matrix<double,4,1>_FootSignal_Percent;
    Eigen::Matrix<double,num_leg,1> _FootSignal_Percent;
};

class xzb_leg_odom {
    private:
        const double Dt=0.002;
        const double mass=15.15;
        // const double num_leg=6;
        const double num_jointperleg=3;

        Eigen::Matrix<double,3,1>COMposition;
        Eigen::Matrix<double,3,1>COMvelocity;


        Eigen::Matrix<double,3,3>R_rot;//


        double num_touching;//估计的触地腿数

        Eigen::VectorXd  LGRF;
        Eigen::VectorXd  GRF;
        Eigen::Vector4i Contactsignal;

        Eigen::Matrix<double,15,1> State_est, State_dr;
        Eigen::Matrix<double,15,15> Mat_cov_P;                       
        Eigen::Matrix<double,15,15> Mat_cov_Q;                        
        Eigen::Matrix<double,3 , 3> Mat_cov_R; 

        double beita[2], beita0[2];                
    
        Eigen::Vector3d noise_gyro, bias_gyro, noise_acc, bias_acc; 
        double gyro_bias_sigma;//标准差
        double acc_bias_sigma;
        double gyro_noise_sigma;
        double acc_noise_sigma;

        //声明函数
        //
        void estimation_EKF(Eigen::Matrix<double,15,1> &X_State);
        void basespeedpredict(); 
        void motion_model(Eigen::Matrix<double,15,1> &X_State,Eigen::MatrixXd F,Eigen::MatrixXd B); 
        void jacobi_f(Eigen::Matrix<double,15,1> X_State,Eigen::MatrixXd &J) ;
        void jacobi_b(Eigen::Matrix<double,15,1> X_State,Eigen::MatrixXd &J) ;
        void jacobi_h(Eigen::MatrixXd &J);
        void observation_model(Eigen::Matrix<double,15,1> X_State,Eigen::Matrix<double,3,1> &Base_vel_observe,Eigen::MatrixXd H);      
        void IMUnoise() ;
        void rpyToR(Eigen::Matrix3d& R, Eigen::Vector3d rpy_in) ;
        void GetMat_Cov_Q(Eigen::Matrix<double,15,6>  J);
        void Setbeita(double Fz_50per[2],double Fz_99per[2]);
        Eigen::Matrix3d getRoMatrix(double r,double p,double y);

    public:
        Eigen::Matrix<double,3,1>COMvelocity_touchpredict;
        Eigen::Matrix<double,3,1>LCOMvelocity_touchpredict;//xzb230512
        Eigen::MatrixXd FootRealPosition;//base
        Eigen::MatrixXd FootRealVelocity;//base  //xzb230512


        xzb_leg_odom();
        BodyStateData_LegOdom bodystatedata;

        //  用来观察
        Eigen::Matrix<double,6,1> IMUData;
        Eigen::MatrixXd Footsignal;
        // Eigen::Matrix<double,3,3>_Jac[num_leg];
        Eigen::Matrix<double,3,1>vcom_touch;


        //声明函数
        //接口函数！
        void legOdom_EKF(Eigen::Vector3d sensor_acc,
                        Eigen::Vector3d sensor_rpydot,
                        Eigen::Matrix<double,num_leg*3,1> f_grf,
                        Eigen::Matrix<double,num_leg*3,1>f_grf_last,
                        Eigen::Matrix<double,num_leg*3,1>  foo_act_pos_ref_body,
                        Eigen::Matrix<double,num_leg*3,1> foo_act_vel_ref_body
                        );
        //接口函数，传入cpgy替代触地    //xzb230512  ——实体上需要删除
        Eigen::Matrix<double,6,1> cpgy;
        void legOdom_cpgy(Eigen::Matrix<double,6,1> _cpgy);

};



#endif