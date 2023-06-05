#include "state_estimator_yxy.h"
#include <pthread.h>
#include <unistd.h>

// struct state_
// {
//     double estimated_contacts[6];
//     Eigen::Matrix<double,3,1> estimated_root_pos;
//     Eigen::Matrix<double,3,1> estimated_root_vel;
//     Eigen::Matrix<double,3,1> root_pos;
//     Eigen::Matrix<double,3,1> root_lin_vel;
// }state;

// 构造函数，对常数矩阵进行初始化
A1BasicEKF::A1BasicEKF () 
{
    // constructor
    eye3.setIdentity();
    // C is fixed
    C.setZero();    //! 测量矩阵C 固定  ->H
    for (int i=0; i<NUM_LEG; ++i) 
    {
        C.block<3,3>(i*3,0) = -eye3;  //-pos
        C.block<3,3>(i*3,6+i*3) = eye3;  //foot pos
        C.block<3,3>(NUM_LEG*3+i*3,3) = eye3;  // vel
        C(NUM_LEG*6+i,6+i*3+2) = 1;  // height z of foot
    }

    // Q R are fixed
    //! 预测协方差矩阵Q 固定
    Q.setIdentity();
    Q.block<3,3>(0,0) = PROCESS_NOISE_PIMU*eye3;               // position transition
    Q.block<3,3>(3,3) = PROCESS_NOISE_VIMU*eye3;               // velocity transition
    for (int i=0; i<NUM_LEG; ++i) {
        Q.block<3,3>(6+i*3,6+i*3) = PROCESS_NOISE_PFOOT*eye3;  // foot position transition
    }

    //! 测量协防差矩阵R 固定
    R.setIdentity();
    for (int i=0; i<NUM_LEG; ++i) 
    {
        R.block<3,3>(i*3,i*3) = SENSOR_NOISE_PIMU_REL_FOOT*eye3;                        // fk estimation
        R.block<3,3>(NUM_LEG*3+i*3,NUM_LEG*3+i*3) = SENSOR_NOISE_VIMU_REL_FOOT*eye3;      // vel estimation
        R(NUM_LEG*6+i,NUM_LEG*6+i) = SENSOR_NOISE_ZFOOT;                               // height z estimation
    }

    // set A to identity
    A.setIdentity();

    // set B to zero
    B.setZero();

    assume_flat_ground = true;  //! 假设平地

}

// //! 不是平地的时候，测量协方差矩阵中对应足端高度z的项直接拉满.先假设地是平的，不管这个
// // A1BasicEKF::A1BasicEKF (bool assume_flat_ground_):A1BasicEKF() 
// // {
// //     // constructor
// //     assume_flat_ground = assume_flat_ground_;
// //     // change R according to this flag, if we do not assume the robot moves on flat ground,
// //     // then we cannot infer height z using this way
// //     if (assume_flat_ground == false) {
// //         for (int i=0; i<NUM_LEG; ++i) {
// //             R(NUM_LEG*6+i,NUM_LEG*6+i) = 1e5;   // height z estimation not reliable
// //         }
// //     }
// // }


void A1BasicEKF::update_estimation(
                            Eigen::Matrix<double,3,1> imu_ang_vel, 
                            Eigen::Matrix<double,3,1> imu_acc,
                            Eigen::Matrix<double,3,3> root_rot_mat,
                            Eigen::Matrix<double,3,6> foot_act_pos_ref_body, 
                            Eigen::Matrix<double,3,6> foot_act_vel_ref_body,
                            double dt,
                            int movement_mode,
                            Eigen::Matrix<double,6,1> touch_dowm_scheduler) 
{
    // _Tim1.update();
    //! 更新状态转移矩阵和输入矩阵 update A B using latest dt
    A.block<3, 3>(0, 3) = dt * eye3;
    B.block<3, 3>(3, 0) = dt * eye3;

    //! 输入是加速度，为（世界坐标系下）机体加速度和重力加速度之和 control input u is Ra + ag
    // Eigen::Vector3d u = root_rot_mat * imu_acc + Eigen::Vector3d(0, 0, -9.81);
    Eigen::Vector3d u = root_rot_mat * imu_acc + Eigen::Vector3d(0, 0, 0);//lcc

    // std::cout<<"u:"<<u.transpose()<<std::endl;
    // std::cout<<"imu_acc:"<<imu_acc.transpose()<<std::endl;
    // std::cout<<"imu_ang_vel:"<<imu_ang_vel.transpose()<<std::endl;

    // contact estimation, do something very simple first
    // if (movement_mode == 0) 
    // {  //! stand状态下，四个足端都是触地的
    //     for (int i = 0; i < NUM_LEG; ++i) 
    //         estimated_contacts[i] = 1.0;
    // } 
    // else 
    // {  //! walk状态下，根据足端受力大小判断
    //     for (int i = 0; i < NUM_LEG; ++i) 
    //     {
    //         //lcc foot_force_z=0~100; 
    //         //    摆动时，foot_force=0~10时，estimated_contacts＝0;
    //         //    支撑时，foot_force=时，estimated_contacts＝0;
    //         estimated_contacts[i] = std::min( std::max( (foot_force_z(i)) / (100.0 - 0.0), 0.0), 1.0 );  
            
    //         //lcc sigmoid函数: 摆动相时estimated_contacts＝１；支撑相时estimated_contacts返回０;
    //         // estimated_contacts[i] = 1.0/(1.0+std::exp(-(foot_force_z(i)-100))); 

    //         //lcc 20230424: 我自己简单加了一个触地判断
    //         //lcc 摆动时，estimated_contacts=0l;
    //         //lcc 支撑时，estimated_contacts=1;
    //         if(touch_dowm_scheduler(i))
    //         {
    //             estimated_contacts[i] = std::min( std::max( (foot_force_z(i)) / (100.0 - 0.0), 0.0), 1.0 );  
    //         }
    //     }
    // }

    //lcc 20230424: 我自己简单加了一个触地判断,没用到足端力
    //         //lcc 摆动时，estimated_contacts=0;
    //         //lcc 支撑时，estimated_contacts=1;
    if(movement_mode=='w' || movement_mode=='q' || movement_mode=='e' ||   //lcc　表示机器人运动模式
        movement_mode=='a' || movement_mode=='s' || movement_mode=='d' )
    {  
        // printf("move mode\n");
        for (int i = 0; i < NUM_LEG; ++i) 
        {
            if(touch_dowm_scheduler(i)==0) //lcc 20230428: 在cpg的touch_dowm_scheduler中，　=0时支撑态;  =1 是摆动态；
            {
                estimated_contacts[i] =1;     //lcc 支撑时，estimated_contacts=1; 
            }
            else
            {
                estimated_contacts[i] =0;   //lcc 摆动时，estimated_contacts=0;
            }
        }    
    }
    else if(movement_mode=='c')  //lcc　机器人蹲
    {   
        // printf("squat mode\n");
        for (int i = 0; i < NUM_LEG; ++i) 
        {
            estimated_contacts[i] =0;     //lcc 摆动时，estimated_contacts=0;
        }  
    }
    else                        //lcc　机器人站立，六腿着地
    {   
        // printf("stand mode\n");
        for (int i = 0; i < NUM_LEG; ++i) 
        {
            estimated_contacts[i] =1;     //lcc 支撑时，estimated_contacts=1; 
        }  
    }


    //! 更新预测协方差Q update Q
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;
    // update Q R for legs not in contact
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        //! 对于没有触地的足端，预测协方差矩阵中对应足端位置的项的协方差拉满　　//lcc 1e3=10^3；　　
        //lcc 摆动时，estimated_contacts=0,(1 - estimated_contacts[i]) * 1e3=1e3,直接拉满；　
        //lcc 支撑时，estimated_contacts=1,(1 - estimated_contacts[i]) * 1e3=0；　
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * dt * PROCESS_NOISE_PFOOT * eye3;  // foot position transition
        // for estimated_contacts[i] == 1, Q = 0.002
        // for estimated_contacts[i] == 0, Q = 1001*Q

        //! 对于没有触地的足端，测量协方差矩阵中对应residual(质心位置和足端位置之差)的项的协方差拉满。 
        R.block<3, 3>(i * 3, i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT * eye3;   
                            // fk estimation
                                                                                                                                                                                                                                                                                                   
        //! 对于没有触地的足端，测量协方差矩阵中对应足端速度的项的协方差拉满。
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3;      // vel estimation
        
        if (assume_flat_ground) 
        {
            //! 在平地上走的前提下。如果足端没有触地，测量协方差矩阵中对应足端高度的项的协方差拉满。
            R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_ZFOOT;       // height z estimation
        }
    }

    //! 预测过程。得到状态先验估计和先验估计协方差矩阵 process update
    xbar = A*x + B*u ;
    Pbar = A * P * A.transpose() + Q;

    // measurement construction
    yhat = C*xbar;
//    leg_v = (-J_rf*av-skew(omega)*p_rf);
//    r((i-1)*3+1:(i-1)*3+3) = body_v - R_er*leg_v;

    //! 实际测量过程 actual measurement
    for (int i=0; i<NUM_LEG; ++i) 
    { 
        Eigen::Vector3d fk_pos = foot_act_pos_ref_body.block<3,1>(0,i);    //lcc foot_act_pos_ref_body,机身系下足端位置
        y.block<3,1>(i*3,0) = root_rot_mat*fk_pos;   // 世界坐标系下足端的位置fk estimation

        //! 足端在全局坐标系下的速度 = 在body坐标系下的速度 + 质心在全局坐标系下的速度
        Eigen::Vector3d leg_v = -foot_act_vel_ref_body.block<3,1>(0,i) - skew(imu_ang_vel)*fk_pos;    //? 世界坐标系下的足端速度 吗？ //lcc foot_act_pos_ref_body,机身系下足端位置
        
        //! 支撑相的时候就用算出来的足端速度（因为足端的速度是根据足端的位置算出来的，而此时足端的位置是可靠的），摆动相的时候（足端位置和速度不可靠）用质心的速度
        //lcc 摆动时，estimated_contacts=0,(1 - estimated_contacts[i]) * 1e3=1e3,直接拉满；　
        //lcc 支撑时，estimated_contacts=1,(1 - estimated_contacts[i]) * 1e3=0；　
        y.block<3,1>(NUM_LEG*3+i*3,0) = (1.0-estimated_contacts[i])*x.segment<3>(3) +  estimated_contacts[i]*root_rot_mat*leg_v;      // vel estimation

        //! 支撑相的时候默认足端高度是0（这个假设yxy的论文中提到过）,摆动相的时候通过质心高度和编码器得到的足端位置计算（加入足端z，是因为他会对质心的运动状态有影响）
        //lcc 摆动时，estimated_contacts=0,(1 - estimated_contacts[i]) * 1e3=1e3,直接拉满；　
        //lcc 支撑时，estimated_contacts=1,(1 - estimated_contacts[i]) * 1e3=0；　
        y(NUM_LEG*6+i) = (1.0-estimated_contacts[i])*(x(2)+fk_pos(2)) + estimated_contacts[i]*0;                               // height z estimation
    }
    //这边的S矩阵是等会用来计算卡尔曼增益的一个trick，并不影响理解 Innovation (or pre-fit residual) covariance 
    S = C * Pbar *C.transpose() + R;
    S = 0.5*(S+S.transpose());//lcc  如果S是迹的话，那这玩意一大串加起来就是他自己Ｓ本身

    error_y = y - yhat; //! 测量误差 = 实际测量值 - 测量矩阵×先验状态  //lcc error_y=y-c*xbar
    Serror_y = S.fullPivHouseholderQr().solve(error_y);     //! 这一项是:（z-Hx）*（H×P×HT + R）卡尔曼增益的分母 //lcc Serror_y=s^-1 *error_y

    x = xbar + Pbar * C.transpose() * Serror_y; //! 后验状态估计   //lcc 20230423:把以上所有公式带入，就是标准的后验公式　没错！！！

    //! 这个S矩阵奇奇怪怪的。反正就是算出来了后验估计协方差
    SC = S.fullPivHouseholderQr().solve(C);    //lcc SC=S^-1 *C
    P = Pbar - Pbar * C.transpose() * SC * Pbar;  //lcc 20230423: 标准的协方差更新公式，没错！！！
    P = 0.5 * (P + P.transpose());  //lcc  如果Ｐ是迹的话，那这玩意一大串加起来就是他自己Ｐ本身
    // cout << "A1BasicEKF time:" << _Tim1.getTimerMilliSec()-1 <<"ms     "<< endl;

    // mat.transpose()：转置矩阵。
    // mat.inverse()：逆矩阵
    // mat.conjugate()：共轭矩阵
    // mat.adjoint()：伴随矩阵
    // mat.trace()：矩阵的迹
    // mat.eigenvalues()：矩阵的特征值
    // mat.determinant()：矩阵求行列式的值
    // mat.diagonal()：矩阵对角线元素
    // mat.sum()：矩阵所有元素求和
    // mat.prod()：矩阵所有元素求积
    // mat.mean()：矩阵所有元素求平均
    // mat.maxCoeff() :矩阵的最大值
    // mat.minCoeff(): 矩阵的最小值
    // mat.mean():矩阵的平均值
    // Matrix::Identity():单位矩阵

    //! tricks：减少位置漂移 reduce position drift
    if (P.block<2, 2>(0, 0).determinant() > 1e-6)   //lcc 没搞懂怎么减少飘移
    {
        P.block<2, 16>(0, 2).setZero();
        P.block<16, 2>(2, 0).setZero();
        P.block<2, 2>(0, 0) /= 10.0;
    }

    // final step
    // put estimated values back to A1CtrlStates& state
    //! 是大于50N才是接触
    //lcc :把上面源代码中由力大小估计点触底状态，返回到state中;
    // for (int i = 0; i < NUM_LEG; ++i) 
    // {
    //     if (estimated_contacts[i] < 0.5) 
    //     {
    //         state.estimated_contacts[i] = false;
    //     } 
    //     else 
    //     {
    //         state.estimated_contacts[i] = true;
    //     }
    // }

    //lcc :把估计结果，返回到state中;
    //    std::cout << x.transpose() <<std::endl;
    state.estimated_root_pos = x.segment<3>(0);     //! 状态估计的得到质心位置
    state.estimated_root_lin_vel = x.segment<3>(3);     //! 状态估计得到的质心速度

    state.estimated_foot_pos_ref_world.block<3,1>(0,0)=x.segment<3>(6);                                       
    state.estimated_foot_pos_ref_world.block<3,1>(0,1)=x.segment<3>(9);                                       
    state.estimated_foot_pos_ref_world.block<3,1>(0,2)=x.segment<3>(12);                                       
    state.estimated_foot_pos_ref_world.block<3,1>(0,3)=x.segment<3>(15);                                       
    state.estimated_foot_pos_ref_world.block<3,1>(0,4)=x.segment<3>(18);                                       
    state.estimated_foot_pos_ref_world.block<3,1>(0,5)=x.segment<3>(21);                                       
    // state.root_pos = x.segment<3>(0);
    // state.root_lin_vel = x.segment<3>(3);

    // std::cout<<"状态估计的得到质心位置:"<<std::endl;
    // std::cout<<x.segment<3>(0).transpose()<<std::endl;

    // std::cout<<"状态估计得到的质心速度:"<<std::endl;
    // std::cout<<x.segment<3>(3).transpose()<<std::endl;
}


// void *state_estimator_thread(void* arg)
// {	
//         printf("remote_monitor thread is :%lu,pid:%d\n",pthread_self(),getpid());
//         while(1)
//         {	

//         }
// }

// 初始化状态空间
void A1BasicEKF::init_state(Eigen::Matrix<double,3,6> foot_act_pos_ref_body,Eigen::Matrix<double,3,3>root_rot_mat) 
{
    // filter_initialized = true;

    //! 先验估计协方差初始化
    P.setIdentity();
    P = P * 3;

    //! 状态先验估计 set initial value of x
    x.setZero();
    Eigen::Matrix<double,3,1> pos=foot_act_pos_ref_body.block<3, 1>(0,0);
    // x.segment<3>(0) << 0, 0, -pos(2);  //! 质心位置初始化，有个高度初值　　　
    x.segment<3>(0) << 0, 0, 0;  //! 质心位置初始化，有个高度初值　　　
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        Eigen::Vector3d fk_pos = foot_act_pos_ref_body.block<3, 1>(0, i);  //lcc foot_act_pos_ref_body,机身系下足端位置

        //lcc o_P_fi = o_R_b * b_P_fi + o_P_com
        x.segment<3>(6 + i * 3) = root_rot_mat * fk_pos + x.segment<3>(0);    //! 阻断在世界坐标系下  root_rot_mat旋转矩阵
    }

    // pthread_t STAEST_THR;
    // pthread_create(&STAEST_THR,NULL,state_estimator_thread,NULL);

}