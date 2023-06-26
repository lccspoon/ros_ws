/* 
 *Last editing: 2022/5/27
 *xzb
 */
#include <random>

#include "xzb_leg_odometer.h"
#include<iostream>
#include<math.h>

using namespace std;
using namespace Eigen;


xzb_leg_odom::xzb_leg_odom(){
cout<<"xzb_leg_odom was successfully called"<<endl;

    Footsignal.resize(num_leg,1);
    Footsignal.setOnes();

    COMvelocity_touchpredict.setZero();
    LCOMvelocity_touchpredict.setZero();

    FootRealPosition.resize(num_leg*num_jointperleg,1);
    FootRealPosition.setZero();
    FootRealVelocity.resize(num_leg*num_jointperleg,1);
    FootRealVelocity.setZero();


    R_rot.setIdentity();   
    LGRF.resize(num_leg*num_jointperleg);
    GRF.resize(num_leg*num_jointperleg);

    Eigen::Vector3d grf0;
    grf0<<0, 0, mass*9.8 / num_leg;
    cout<<" Initialization 0000! "<<endl;

    for (int i = 0; i < num_leg; i++)
    {
        GRF.segment(3*i,3)=grf0;
        LGRF.segment(3*i,3)=grf0;
        bodystatedata._GRF.segment(3*i,3)=grf0;
    }
    cout<<" Initialization 11111! "<<endl;

    IMUData.setZero();
    State_est.setZero();
    State_est(2)=0.45;
    State_dr.setZero();
    State_dr(2)=0.45;
    Mat_cov_P=MatrixXd::Identity(15,15);
    Mat_cov_R=Matrix3d::Identity(3,3);//测量协方差矩阵
    Mat_cov_Q=MatrixXd::Identity(15,15);
    cout<<" Initialization 222222! "<<endl;
    

    //nosie
    gyro_bias_sigma = 10*M_PI/(180*60*60);
    acc_bias_sigma = (15e-06)*9.8;
    gyro_noise_sigma = 0.01*M_PI/180;
    acc_noise_sigma = (60e-06)*9.8;

    bodystatedata._pCOM.setZero();
    bodystatedata._vCOM.setZero();
    bodystatedata._aCOM.setZero();
    bodystatedata._rpyCOM.setZero();
    bodystatedata._rpydotCOM.setZero();
    bodystatedata._FootSignal_Percent.setOnes();

    double fz_50per[2], fz_99per[2];  //0为三足触地；1为六足触地
    fz_50per[0]=50;fz_50per[1]=25;  //传感器矫正后， fz
    fz_99per[0]=80;fz_99per[1]=50;    

    Setbeita(fz_50per,fz_99per);

    cout<<" Initialization succeeded! "<<endl;
              
}

void xzb_leg_odom::legOdom_EKF(Eigen::Vector3d sensor_acc,
                        Eigen::Vector3d sensor_rpydot,
                        Eigen::Matrix<double,num_leg*3,1> f_grf,
                        Eigen::Matrix<double,num_leg*3,1>f_grf_last,
                        Eigen::Matrix<double,num_leg*3,1>  foo_act_pos_ref_body,
                        Eigen::Matrix<double,num_leg*3,1> foo_act_vel_ref_body
                        )
{
    // cout<<"COMvelocity_touchpredict"<<COMvelocity_touchpredict.transpose()<<endl;

    IMUData<<sensor_acc,sensor_rpydot;
    // IMUnoise(); //暂时用不上
    // bodystatedata._aCOM=sensor_acc+bias_acc+noise_acc;//添加噪声  //暂时用不上
    bodystatedata._aCOM=sensor_acc;
    bodystatedata._rpydotCOM=sensor_rpydot;

    FootRealVelocity=foo_act_vel_ref_body;//base 
    FootRealPosition=foo_act_pos_ref_body;
    LGRF=f_grf_last;
    GRF=f_grf;

    // cout<<"start estimation_EKF"<<endl;

    estimation_EKF(State_est);
    // cout<<"finish estimation_EKF"<<endl;


    vcom_touch=COMvelocity_touchpredict;//将运动学速度输出plot

    // cout<<"State_est"<<State_est.transpose()<<endl;

    bodystatedata._pCOM=State_est.block(0,0,3,1);
    bodystatedata._vCOM=State_est.block(3,0,3,1);
    bodystatedata._rpyCOM=State_est.block(6,0,3,1);

}


void xzb_leg_odom::estimation_EKF(Matrix<double,15,1> &X_State) 
{
	// cout<<"estimation_EKF"<<endl;

    rpyToR(R_rot,bodystatedata._rpyCOM);
    Matrix<double,3,1> Base_vel;
    // cout<<"start basespeedpredict"<<endl;
    basespeedpredict(); 
    // cout<<"finish basespeedpredict"<<endl;

    Base_vel=COMvelocity_touchpredict;

  
//predict
    MatrixXd JB(15,6);
    jacobi_b(X_State,JB);
    GetMat_Cov_Q(JB);
    MatrixXd JF(15,15);
    jacobi_f(X_State , JF);
	// cout<<"X_State_before: "<<X_State.transpose()<<endl; 

    motion_model( X_State,JF,JB);
    Mat_cov_P=JF*Mat_cov_P*JF.transpose()+Mat_cov_Q;
	// cout<<"Mat_cov_Q: "<<endl<<Mat_cov_Q<<endl; 

//update
    MatrixXd JH;
    jacobi_h(JH);
    Vector3d z_pred;
    observation_model(X_State,z_pred,JH);
    Vector3d y=Base_vel-z_pred;
	// cout<<"z_pred"<<z_pred.transpose()<<endl; 
    Matrix3d S=JH*Mat_cov_P*JH.transpose()+Mat_cov_R; 
	// cout<<"Mat_cov_R:  "<<endl<<Mat_cov_R<<endl; 
	// cout<<"S:  "<<endl<<S<<endl; 
    if (S.determinant()==0)
    {
        cout<<"S 矩阵奇异!!!!!!"<<endl;
    }
    MatrixXd K=Mat_cov_P*JH.transpose()*S.inverse();
	// cout<<"K:  "<<endl<<K<<endl; 
    X_State=X_State+K*y;
	// cout<<"X_State_affter: "<<X_State.transpose()<<endl; 

    Mat_cov_P=(MatrixXd::Identity(15,15)-K*JH)*Mat_cov_P;
}


void xzb_leg_odom::basespeedpredict()//根据触地判断计算机身速度
{
	// cout<<"basespeedpredict"<<endl; 

    Vector3d bwb = bodystatedata._rpydotCOM;
    Matrix<double,num_leg,1> Pk = Matrix<double,num_leg,1>::Zero();
             num_touching = num_leg;

    Vector3d b_X_fl, b_dX_fl;
    Matrix<double,3,num_leg> b_dX_bl;
    Vector3d sum_PmuldXl = Vector3d::Zero(3);

    for (int i = 0; i < num_leg; i++) //FR HR HL FL
    {
        int gaittype_index=0;  


        //传入处于xx步态，x条腿触地
        // switch ( Contactsignal.sum() )
        // {
        // // case 1 : gaittype_index=0;
        // case 2 : 
        //     gaittype_index=0;   
        //     break;        
        // // case 3 : gaittype_index=2;
        // case 4 : 
        //     gaittype_index=1;
        //     break;        
        // default : 
        //     gaittype_index=0;  
        //     break;    
        // }
        

        // Pk(i) = 1 / (1 + exp(-beita[gaittype_index] * GRF(3 * i+2) - beita0[gaittype_index])); 
        Pk=cpgy;//xzb230512 由于仿真中无法获得准确的GRF，因此暂时用cpgy信号替代

        

        bodystatedata._FootSignal_Percent(i)=Pk(i);

        if (Pk(i) < 0.5)
        {
            Pk(i) = 0;
            num_touching = num_touching - 1;
        }

        b_X_fl=FootRealPosition.block(3*i,0,3,1);
        b_dX_fl = FootRealVelocity.block(3*i,0,3,1);  

        b_dX_bl.col(i) = -b_dX_fl - bwb.cross(b_X_fl);//.cross叉乘 
        sum_PmuldXl = sum_PmuldXl + b_dX_bl.col(i) * Pk(i);
    }


    double sum_P = Pk.sum();
    if (sum_P<=0)
    {
        cout<<"!!!!!!!!WITHOUT TOUCHING  THE GRAND!!!!!!!!"<<endl;
        if (num_touching==0)
        {
            num_touching=num_leg/2;
            sum_P=num_leg/2;//防止Pk全为0导致错误
        }
        COMvelocity_touchpredict=LCOMvelocity_touchpredict;//防止Pk全为0导致错误
    }
    else
    {
        // cout<<"sum_P  OKOKOKOK"<<endl;
        COMvelocity_touchpredict = sum_PmuldXl / sum_P;
        LCOMvelocity_touchpredict=COMvelocity_touchpredict;
    }
    // cout<<"COMvelocity_touchpredict:       "<< COMvelocity_touchpredict.transpose() <<endl;



    //  观测协方差矩阵Mat_cov_R的计算              
    Matrix3d Matrix_sum_delta2_dX = Matrix3d::Zero(3,3);
    Matrix3d Matrix_delta2_dX;
    Vector3d delta_dX;
    double sum_f = 0;
    double delta_f;
    double alpha = 10;

                                    
    Matrix3d R_sqrt;

    for (int i = 0; i < num_leg; i++)
    {
        if (Pk(i) >= 0.5)
        {
            Pk(i) = 1;
            Footsignal(i)=1;
        }
        else
        {   Pk(i) = 0;
            Footsignal(i)=0;
        }
        delta_dX = COMvelocity_touchpredict - b_dX_bl.col(i);   

        Vector3d delta_dX2= delta_dX.array().square();
        Matrix3d Delta_dX2=delta_dX2.asDiagonal();
        Matrix_sum_delta2_dX = Matrix_sum_delta2_dX + Delta_dX2* Pk(i); 
        // cout<<"Matrix_sum_delta2_dX"<<i<<":       "<< Matrix_sum_delta2_dX <<endl;

        sum_f = sum_f + abs(GRF(3 * i+2) - LGRF(3 * i+2)) * Pk(i);    //全局变量
        // cout<<"sum_f"<<i<<":       "<< sum_f <<endl;
    }


    Matrix_delta2_dX = Matrix_sum_delta2_dX/num_touching;
    delta_f = sum_f/num_touching;
    // cout<<"Matrix_sum_delta2_dX:       "<< Matrix_sum_delta2_dX <<endl;
    // cout<<"num_touching:       "<< num_touching <<endl;
    // cout<<"Matrix_delta2_dX:       "<< Matrix_delta2_dX <<endl;


    Matrix3d Matrix_delta2_dX_sqrt = Matrix_delta2_dX.array().sqrt();

    // R_sqrt = (alpha1*Matrix_delta2_dX_sqrt + (1-alpha1)*alpha2*delta_f * Matrix3d::Identity(3, 3));//参考17年
    R_sqrt = 0.25 * (Matrix_delta2_dX_sqrt + (delta_f / alpha) * Matrix3d::Identity(3, 3));//参考20年pronto
    // cout<<"R_sqrt:       "<< R_sqrt <<endl;

    Mat_cov_R = R_sqrt.array().square(); //3*3
    // cout<<"Mat_cov_R:       "<< Mat_cov_R <<endl;

}


void xzb_leg_odom::motion_model(Matrix<double,15,1> &X_State,MatrixXd F,MatrixXd B)
{
    // cout<<"motion_model!!!!"<<endl;
    X_State=F*X_State+B*IMUData;    
    //加个非线性的！！！！
    // cout<<"motion_model    OKOKOKOK"<<endl;
}

void xzb_leg_odom::jacobi_f(Matrix<double,15,1> X_State,MatrixXd &J) 
{ 
    // cout<<"JF!!!!!!!!!!!!!"<<endl;
    Matrix3d dJF13dtheta,dJF33dtheta;
   //0819
    dJF13dtheta <<   
    (pow(Dt,2)*(cos(X_State(6))*sin(X_State(8)) - cos(X_State(8))*sin(X_State(7))*sin(X_State(6))))/2 + (pow(Dt,2)*(sin(X_State(6))*sin(X_State(8)) + cos(X_State(6))*cos(X_State(8))*sin(X_State(7))))/2, 
    (pow(Dt,2)*cos(X_State(7))*cos(X_State(6))*cos(X_State(8)))/2 - (pow(Dt,2)*cos(X_State(8))*sin(X_State(7)))/2 + (pow(Dt,2)*cos(X_State(7))*cos(X_State(8))*sin(X_State(6)))/2, 
    (pow(Dt,2)*(cos(X_State(8))*sin(X_State(6)) - cos(X_State(6))*sin(X_State(7))*sin(X_State(8))))/2 - (pow(Dt,2)*(cos(X_State(6))*cos(X_State(8)) + sin(X_State(7))*sin(X_State(6))*sin(X_State(8))))/2 - (pow(Dt,2)*cos(X_State(7))*sin(X_State(8)))/2,
    (pow(Dt,2)*cos(X_State(7))*cos(X_State(6)))/2 - (pow(Dt,2)*cos(X_State(7))*sin(X_State(6)))/2,                    
    - (pow(Dt,2)*cos(X_State(7)))/2 - (pow(Dt,2)*cos(X_State(6))*sin(X_State(7)))/2 - (pow(Dt,2)*sin(X_State(7))*sin(X_State(6)))/2,
    0,
    - (pow(Dt,2)*(cos(X_State(6))*cos(X_State(8)) + sin(X_State(7))*sin(X_State(6))*sin(X_State(8))))/2 - (pow(Dt,2)*(cos(X_State(8))*sin(X_State(6)) - cos(X_State(6))*sin(X_State(7))*sin(X_State(8))))/2,
    (pow(Dt,2)*cos(X_State(7))*cos(X_State(6))*sin(X_State(8)))/2 - (pow(Dt,2)*sin(X_State(7))*sin(X_State(8)))/2 + (pow(Dt,2)*cos(X_State(7))*sin(X_State(6))*sin(X_State(8)))/2,
    (pow(Dt,2)*(sin(X_State(6))*sin(X_State(8)) + cos(X_State(6))*cos(X_State(8))*sin(X_State(7))))/2 - (pow(Dt,2)*(cos(X_State(6))*sin(X_State(8)) - cos(X_State(8))*sin(X_State(7))*sin(X_State(6))))/2 + (pow(Dt,2)*cos(X_State(7))*cos(X_State(8)))/2;
    
    dJF33dtheta<<Matrix3d::Identity();

    J.resize(15,15);
    // 0819
    J<<Matrix3d::Identity(),    Dt*R_rot,               dJF13dtheta,         Matrix3d::Zero(),      Matrix3d::Zero(),
        Matrix3d::Zero(),       Matrix3d::Identity(),   Matrix3d::Zero(),    Matrix3d::Zero(),      Matrix3d::Zero(), 
        Matrix3d::Zero(),       Matrix3d::Zero(),       Matrix3d::Identity(),         Matrix3d::Zero(),      Matrix3d::Zero(), 
        Matrix3d::Zero(),       Matrix3d::Zero(),       Matrix3d::Zero(),   Matrix3d::Identity(),   Matrix3d::Zero(),   
        Matrix3d::Zero(),       Matrix3d::Zero(),       Matrix3d::Zero(),   Matrix3d::Zero(),       Matrix3d::Identity();

// // 添加噪声后需要修改为以下形式
    // J<<Matrix3d::Identity(),    Dt*R_rot.transpose(),   dJF13dtheta,         0.5*pow(Dt,2)*R_rot.transpose(),      Matrix3d::Zero(),
    //     Matrix3d::Zero(),       Matrix3d::Identity(),   Matrix3d::Zero(),    Dt*Matrix3d::Identity(),              Matrix3d::Zero(), 
    //     Matrix3d::Zero(),       Matrix3d::Zero(),       dJF33dtheta,         Matrix3d::Zero(),                     Dt*R_rot.transpose(),
    //     Matrix3d::Zero(),       Matrix3d::Zero(),       Matrix3d::Zero(),   Matrix3d::Identity(),                  Matrix3d::Zero(),   
    //     Matrix3d::Zero(),       Matrix3d::Zero(),       Matrix3d::Zero(),   Matrix3d::Zero(),                      Matrix3d::Identity();


    // cout<<"JF   OKOKOKOk"<<endl;
}


void xzb_leg_odom::jacobi_b(Matrix<double,15,1> X_State,MatrixXd &J)
{   
    // cout<<"JB!!!!!!!!!!!!!"<<endl;
    J.resize(15,6);
/*  //0819
    J<<0.5*pow(Dt,2)*R_rot,                 Matrix3d::Zero(),
        Dt*Matrix3d::Identity(),            Matrix3d::Zero(),
        Matrix3d::Zero(),                   Dt*R_rot,
        Matrix3d::Zero(),                   Matrix3d::Zero(),
        Matrix3d::Zero(),                   Matrix3d::Zero(); */
    // //1022
    //     J<<Matrix3d::Zero(),                Matrix3d::Zero(),
    //     Dt*Matrix3d::Identity(),            Matrix3d::Zero(),
    //     Matrix3d::Zero(),                   Dt*R_rot,
    //     Matrix3d::Zero(),                   Matrix3d::Zero(),
    //     Matrix3d::Zero(),                   Matrix3d::Zero();
    //1022
        J<<0.5*pow(Dt,2)*R_rot,                Matrix3d::Zero(),
            Dt*Matrix3d::Identity(),            Matrix3d::Zero(),
            Matrix3d::Zero(),                   Dt*R_rot,
            Matrix3d::Identity(),               Matrix3d::Zero(),
            Matrix3d::Zero(),                   Matrix3d::Identity();
    // cout<<"JB   OKOKOKOk"<<endl;

}



void xzb_leg_odom::jacobi_h(MatrixXd &J) 
{   J.resize(3,15);
    J <<Matrix3d::Zero(),Matrix3d::Identity() ,Matrix3d::Zero(), Matrix3d::Zero(),Matrix3d::Zero();
}

void xzb_leg_odom::observation_model(Matrix<double,15,1> X_State,
                                            Matrix<double,3,1> &Base_vel_observe,MatrixXd H)
{ 
     Base_vel_observe=H*X_State;
}

void xzb_leg_odom::IMUnoise()//IMU噪声更新  
{
    random_device rd;                        //产生随机数
    default_random_engine generator_(rd());  //创建随机数引擎对象
    normal_distribution<double> noise(0, 1); //正态分布对象

    Vector3d random_noise_gyro(noise(generator_), noise(generator_), noise(generator_));
    Vector3d random_noise_acc(noise(generator_), noise(generator_), noise(generator_));
    Vector3d random_noise_gyro_bias(noise(generator_), noise(generator_), noise(generator_));
    Vector3d random_noise_acc_bias(noise(generator_), noise(generator_), noise(generator_));

    Matrix3d gyro_sqrt_cov = gyro_noise_sigma * Matrix3d::Identity();
    Matrix3d acc_sqrt_cov = acc_noise_sigma * Matrix3d::Identity();
    
    // gassi noise
    noise_gyro = gyro_sqrt_cov / sqrt(Dt)*random_noise_gyro;
    noise_acc = acc_sqrt_cov  /  sqrt(Dt)* random_noise_acc ;

    // bias update
    bias_gyro += gyro_bias_sigma * sqrt(Dt) * random_noise_gyro_bias;  
    bias_acc += acc_bias_sigma * sqrt(Dt) * random_noise_acc_bias;
    // acc_imudata__act=acc_imudata+noise_acc+bias_acc;
    // gyro_imudata_act=gyro_imudata+noise_gyro+bias_gyro;
}
 
void xzb_leg_odom::rpyToR(Matrix3d& R, Vector3d rpy_in) 
{
  Matrix3d Rz, Ry, Rx;
  R.setIdentity();
  Rz.setIdentity();
  Ry.setIdentity();
  Rx.setIdentity();
  Rz(0, 0) = cos(rpy_in(2));
  Rz(0, 1) = -sin(rpy_in(2));
  Rz(1, 0) = sin(rpy_in(2));
  Rz(1, 1) = cos(rpy_in(2));
  Ry(0, 0) = cos(rpy_in(1));
  Ry(0, 2) = sin(rpy_in(1));
  Ry(2, 0) = -sin(rpy_in(1));
  Ry(2, 2) = cos(rpy_in(1));
  Rx(1, 1) = cos(rpy_in(0));
  Rx(1, 2) = -sin(rpy_in(0));
  Rx(2, 1) = sin(rpy_in(0));
  Rx(2, 2) = cos(rpy_in(0));
  R.block(0,0,3,3) = Rz * Ry * Rx;
}

void xzb_leg_odom::GetMat_Cov_Q(Matrix<double,15,6>  J)
{
   /* // MatrixXd Mat_cov_Q;//过程协方差矩阵
    // Matrix3d cov_bias_acc_sqrt = acc_bias_sigma * sqrt(Dt) * Matrix3d::Identity();
    Matrix3d cov_bias_acc_sqrt = acc_bias_sigma* Matrix3d::Identity();
    Matrix3d cov_bias_acc = cov_bias_acc_sqrt.array().square();
    Matrix3d cov_noise_acc_sqrt = acc_noise_sigma / sqrt(Dt) * Matrix3d::Identity();
    Matrix3d cov_noise_acc = cov_noise_acc_sqrt.array().square();
    // Matrix3d cov_bias_gyro_sqrt = gyro_bias_sigma * sqrt(Dt) * Matrix3d::Identity();
    Matrix3d cov_bias_gyro_sqrt = gyro_bias_sigma * Matrix3d::Identity();
    Matrix3d cov_bias_gyro = cov_bias_gyro_sqrt.array().square();
    Matrix3d cov_noise_gyro_sqrt = gyro_noise_sigma / sqrt(Dt) * Matrix3d::Identity();
    Matrix3d cov_noise_gyro = cov_noise_gyro_sqrt.array().square();

    Matrix<double,6,6> Q;
//0819
    Q<<  cov_bias_acc,              Matrix3d::Zero(),
        Matrix3d::Zero(),           cov_bias_gyro; 
    Mat_cov_Q=J*Q*J.transpose(); 
    //1022  设置Mat_Cov_Q为0
    Q<<  cov_bias_acc,              Matrix3d::Zero(),
        Matrix3d::Zero(),           cov_bias_gyro; 
    Mat_cov_Q=J*Q*J.transpose();    */
//1022
    Matrix<double,15,6>  J2;
        J2<<Matrix3d::Zero(),                Matrix3d::Zero(),
            Dt*Matrix3d::Identity(),            Matrix3d::Zero(),
            Matrix3d::Zero(),                   Dt*R_rot,
            Matrix3d::Identity(),               Matrix3d::Zero(),
            Matrix3d::Zero(),                   Matrix3d::Identity();
    Matrix<double,15,12>  G;
    G<<J,J2;
    Matrix<double,12,12> Q;
    Q<<  15e-6*9.8*Matrix3d::Identity(),    Matrix3d::Zero(),                    Matrix3d::Zero(),       Matrix3d::Zero(),
        Matrix3d::Zero(),      10/180*3.1415926/(60*60)*Matrix3d::Identity(),         Matrix3d::Zero(),       Matrix3d::Zero(),
        Matrix3d::Zero(),       Matrix3d::Zero(),                                Matrix3d::Zero(),       Matrix3d::Zero(),
        Matrix3d::Zero(),       Matrix3d::Zero(),                                Matrix3d::Zero(),           Matrix3d::Zero();
    Mat_cov_Q=G*Q*G.transpose();
}

void xzb_leg_odom::Setbeita(double Fz_50per[2],double Fz_99per[2])
{
    double Pk_99per=0.99;
    for (int i = 0; i < 2; i++)
    {
        beita[i]=(log(1/Pk_99per-1)) /(Fz_50per[i]-Fz_99per[i]);
        beita0[i]=-Fz_50per[i]*beita[i];  
    }
    cout<<"beita:"<<beita[0]<<beita[1]<<"; "<<"beita0:"<<beita0[0]<<beita0[1]<<endl;
}


Matrix3d xzb_leg_odom::getRoMatrix(double r,double p,double y){
    Matrix3d R_rot;
    R_rot<<cos(y)*cos(p), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r) ,
            sin(y)*cos(p), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r) ,
            -sin(p), cos(p)*sin(r), cos(p)*cos(r);
    return R_rot;
}


void xzb_leg_odom::legOdom_cpgy(Eigen::Matrix<double,6,1> _cpgy){
    for (int i = 0; i < num_leg; i++)
    {
        if(_cpgy(i)==0 )
        {
            cpgy(i)=1;
        }
        else
        {
            cpgy(i)=0;
        }
        
    }
    
}