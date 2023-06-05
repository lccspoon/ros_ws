/*! @file SimulationBridge.cpp
*  @brief  The SimulationBridge runs a RobotController and connects it to a
*          Simulator, using shared memory. It is the simulation version of the
*          HardwareBridge.
*/


#include "simulation_bridge.h"
#include "../../user/remote_monitor/keyboard_ctrl.h"
#include "../robot_master/robot_main.h"

int SIM_SEQ;


struct joint_state
{
    int seq_real=0;
    double pos[18]={0},vel[18]={0},eff[18]={0};
}js;

struct angular_velocity
{
    double x,y,z;
};

struct linear_acceleration
{
    double x,y,z;
};

struct quaternion_orientation
{
    double x,y,z,w;
};

struct imu_msg
{
    int seq_real=0;
    quaternion_orientation qua_ori;
    linear_acceleration lin_acc;
    angular_velocity ang_vel;

    double qua_orie[4]={0};
    double lin_acce[3]={0};
    double ang_velo[3]={0};
}imu;

std::mutex mux_joint;
void doMsg_yobotics_joint_states(const sensor_msgs::JointState::ConstPtr& yobotics_joint_states)
{
    mux_joint.lock();

        js.seq_real=yobotics_joint_states->header.seq;
        // printf("seq_real:%d \n",js.seq_real);
        for(int i=0;i<18;i++)
        {
            js.pos[i]=yobotics_joint_states->position[i];
            // printf("[%d]:%f ", i,pos[i]*180/3.1415);
        }
        // printf("vel \n");
        for(int i=0;i<18;i++)
        {
            js.vel[i]=yobotics_joint_states->velocity[i];
            // printf("%f ", vel[i]);
        }
        // printf("eff \n");
        for(int i=0;i<18;i++)
        {
            js.eff[i]=yobotics_joint_states->effort[i];
            // printf("%f ", eff[i]);
        }
    mux_joint.unlock();
}

Eigen::Matrix<double,18,1> ROSLegTopicHandle::retLegMsgPosition()
{
    Eigen::Matrix<double,18,1> pos;
    for (int i = 0; i < 18; i++)
    {
        pos(i)=js.pos[i];
    }
    return pos;
}
Eigen::Matrix<double,18,1> ROSLegTopicHandle::retLegMsgVelocity()
{
    Eigen::Matrix<double,18,1> vel;
    for (int i = 0; i < 18; i++)
    {
        vel(i)=js.vel[i];
    }
    return vel;
}
Eigen::Matrix<double,18,1> ROSLegTopicHandle::retLegMsgTorque()
{
    Eigen::Matrix<double,18,1> tor;
    for (int i = 0; i < 18; i++)
    {
        tor(i)=js.eff[i];
    }
    return tor;
}
int ROSLegTopicHandle::retLegMsgSeq()
{
    return js.seq_real;
}

std::mutex mux_imu;
Eigen::Matrix<double,3,1> imuacc,imuangelvel;
void doMsg_yobotics_imu_msg(const sensor_msgs::Imu::ConstPtr& yobotics_imu_msg)
{
    mux_imu.lock();
    
        SIM_SEQ=imu.seq_real=yobotics_imu_msg->header.seq;
        // cout<<SIM_SEQ<<endl;
        imu.qua_ori.x=yobotics_imu_msg->orientation.x;
        imu.qua_ori.y=yobotics_imu_msg->orientation.y;
        imu.qua_ori.z=yobotics_imu_msg->orientation.z;
        imu.qua_ori.w=yobotics_imu_msg->orientation.w;
        imu.qua_orie[0]=imu.qua_ori.x;
        imu.qua_orie[1]=imu.qua_ori.y;
        imu.qua_orie[2]=imu.qua_ori.z;
        imu.qua_orie[3]=imu.qua_ori.w;

        imu.ang_vel.x=yobotics_imu_msg->angular_velocity.x;
        imu.ang_vel.y=yobotics_imu_msg->angular_velocity.y;
        imu.ang_vel.z=yobotics_imu_msg->angular_velocity.z;
        imu.ang_velo[0]=imu.ang_vel.x;
        imu.ang_velo[1]=imu.ang_vel.y;
        imu.ang_velo[2]=imu.ang_vel.z;
        imuangelvel<<imu.ang_vel.x,imu.ang_vel.y,imu.ang_vel.z;

        imu.lin_acc.x=yobotics_imu_msg->linear_acceleration.x;
        imu.lin_acc.y=yobotics_imu_msg->linear_acceleration.y;
        imu.lin_acc.z=yobotics_imu_msg->linear_acceleration.z;
        imu.lin_acce[0]=imu.lin_acc.x;
        imu.lin_acce[1]=imu.lin_acc.y;
        imu.lin_acce[2]=imu.lin_acc.z;
        imuacc<<imu.lin_acc.x,imu.lin_acc.y,imu.lin_acc.z;

    mux_imu.unlock();
}

Eigen::Matrix<double,4,1> ROSLegTopicHandle::retImuQuaOri()
{
    Eigen::Matrix<double,4,1> qua;
    qua<< imu.qua_orie[0], imu.qua_orie[1], imu.qua_orie[2], imu.qua_orie[3];
    return qua;
}
Eigen::Matrix<double,3,1> ROSLegTopicHandle::retImuAngVel()
{
    Eigen::Matrix<double,3,1> vel;
    vel<< imu.ang_velo[0], imu.ang_velo[1], imu.ang_velo[2];
    return vel;
}

Eigen::Matrix<double,3,1> ROSLegTopicHandle::retImuLinAcc()
{
    Eigen::Matrix<double,3,1> acc;
    acc<< imu.lin_acce[0], imu.lin_acce[1], imu.lin_acce[2];
    return acc;
}

int ROSLegTopicHandle::retImuSeq()
{
    return imu.seq_real;
}


struct nav_odometry
{
   int seq;
   double   position[3];
   double twist_linear[3];
}odometry;

std::mutex mux_odometry;  
void doMsg_nav_odometry(const nav_msgs::Odometry::ConstPtr& nav_odometry )
{
    mux_odometry.lock();
    odometry.seq=nav_odometry->header.seq;

        odometry.position[0]=nav_odometry->pose.pose.position.x;
        odometry.position[1]=nav_odometry->pose.pose.position.y;
        odometry.position[2]=nav_odometry->pose.pose.position.z;

        printf(" odometry.position: %f,  %f,  %f \n",odometry.position[0],odometry.position[1],odometry.position[2]);

        odometry.twist_linear[0]=nav_odometry->twist.twist.linear.x;
        odometry.twist_linear[1]=nav_odometry->twist.twist.linear.y;
        odometry.twist_linear[2]=nav_odometry->twist.twist.linear.z;

    mux_odometry.unlock();
}

Eigen::Vector3d odometry_position;
Eigen::Vector3d ROSLegTopicHandle::retOdoPostion(void)
{
    odometry_position<<odometry.position[0], odometry.position[1], odometry.position[2];
    return odometry_position;
}
double * ROSLegTopicHandle::retOdoTwistLinear(void)
{
    return odometry.twist_linear;
}
int ROSLegTopicHandle::retOdoSeq(void)
{
    return odometry.seq;
}


struct bumper_states
{
   int seq;
   double force[3];
   double torque[3];
}bumper_states_L1,bumper_states_L2,bumper_states_L3,bumper_states_R1,bumper_states_R2,bumper_states_R3;

void doMsg_bumper_L1_states(const geometry_msgs::WrenchStamped::ConstPtr& bumper_states)
{
    // std::cout<<"bumper_states->header.seq: "<<bumper_states->header.seq<<std::endl;
    bumper_states_L1.seq=bumper_states->header.seq;
    bumper_states_L1.force[0]=bumper_states->wrench.force.x;
    bumper_states_L1.force[1]=bumper_states->wrench.force.y;
    bumper_states_L1.force[2]=bumper_states->wrench.force.z;
    bumper_states_L1.torque[0]=bumper_states->wrench.torque.x;
    bumper_states_L1.torque[1]=bumper_states->wrench.torque.y;
    bumper_states_L1.torque[2]=bumper_states->wrench.torque.z;
}
void doMsg_bumper_L2_states(const geometry_msgs::WrenchStamped::ConstPtr& bumper_states)
{
    bumper_states_L2.seq=bumper_states->header.seq;
    bumper_states_L2.force[0]=bumper_states->wrench.force.x;
    bumper_states_L2.force[1]=bumper_states->wrench.force.y;
    bumper_states_L2.force[2]=bumper_states->wrench.force.z;
    bumper_states_L2.torque[0]=bumper_states->wrench.torque.x;
    bumper_states_L2.torque[1]=bumper_states->wrench.torque.y;
    bumper_states_L2.torque[2]=bumper_states->wrench.torque.z;
}
void doMsg_bumper_L3_states(const geometry_msgs::WrenchStamped::ConstPtr& bumper_states)
{
    bumper_states_L3.seq=bumper_states->header.seq;
    bumper_states_L3.force[0]=bumper_states->wrench.force.x;
    bumper_states_L3.force[1]=bumper_states->wrench.force.y;
    bumper_states_L3.force[2]=bumper_states->wrench.force.z;
    bumper_states_L3.torque[0]=bumper_states->wrench.torque.x;
    bumper_states_L3.torque[1]=bumper_states->wrench.torque.y;
    bumper_states_L3.torque[2]=bumper_states->wrench.torque.z;
}
void doMsg_bumper_R1_states(const geometry_msgs::WrenchStamped::ConstPtr& bumper_states)
{
    bumper_states_R1.seq=bumper_states->header.seq;
    bumper_states_R1.force[0]=bumper_states->wrench.force.x;
    bumper_states_R1.force[1]=bumper_states->wrench.force.y;
    bumper_states_R1.force[2]=bumper_states->wrench.force.z;
    bumper_states_R1.torque[0]=bumper_states->wrench.torque.x;
    bumper_states_R1.torque[1]=bumper_states->wrench.torque.y;
    bumper_states_R1.torque[2]=bumper_states->wrench.torque.z;
}
void doMsg_bumper_R2_states(const geometry_msgs::WrenchStamped::ConstPtr& bumper_states)
{
    bumper_states_R2.seq=bumper_states->header.seq;
    bumper_states_R2.force[0]=bumper_states->wrench.force.x;
    bumper_states_R2.force[1]=bumper_states->wrench.force.y;
    bumper_states_R2.force[2]=bumper_states->wrench.force.z;
    bumper_states_R2.torque[0]=bumper_states->wrench.torque.x;
    bumper_states_R2.torque[1]=bumper_states->wrench.torque.y;
    bumper_states_R2.torque[2]=bumper_states->wrench.torque.z;
}
void doMsg_bumper_R3_states(const geometry_msgs::WrenchStamped::ConstPtr& bumper_states)
{
    bumper_states_R3.seq=bumper_states->header.seq;
    bumper_states_R3.force[0]=bumper_states->wrench.force.x;
    bumper_states_R3.force[1]=bumper_states->wrench.force.y;
    bumper_states_R3.force[2]=bumper_states->wrench.force.z;
    bumper_states_R3.torque[0]=bumper_states->wrench.torque.x;
    bumper_states_R3.torque[1]=bumper_states->wrench.torque.y;
    bumper_states_R3.torque[2]=bumper_states->wrench.torque.z;
}

Eigen::Matrix<double,3,6> ROSLegTopicHandle::retFootEndForec()
{
    Eigen::Vector3d forec;
    forec(0)=bumper_states_L1.force[0]; 
    forec(1)=bumper_states_L1.force[1]; 
    forec(2)=bumper_states_L1.force[2]; 
    foot_end_force.block<3,1>(0,0)=forec;
    forec(0)=bumper_states_L2.force[0]; 
    forec(1)=bumper_states_L2.force[1]; 
    forec(2)=bumper_states_L2.force[2]; 
    foot_end_force.block<3,1>(0,1)=forec;
    forec(0)=bumper_states_L3.force[0]; 
    forec(1)=bumper_states_L3.force[1]; 
    forec(2)=bumper_states_L3.force[2]; 
    foot_end_force.block<3,1>(0,2)=forec;

    forec(0)=bumper_states_R1.force[0]; 
    forec(1)=bumper_states_R1.force[1]; 
    forec(2)=bumper_states_R1.force[2]; 
    foot_end_force.block<3,1>(0,3)=forec;
    forec(0)=bumper_states_R2.force[0]; 
    forec(1)=bumper_states_R2.force[1]; 
    forec(2)=bumper_states_R2.force[2]; 
    foot_end_force.block<3,1>(0,4)=forec;
    forec(0)=bumper_states_R3.force[0]; 
    forec(1)=bumper_states_R3.force[1]; 
    forec(2)=bumper_states_R3.force[2]; 
    foot_end_force.block<3,1>(0,5)=forec;

    // std::cout<<"foot_end_force: "<<std::endl;
    // std::cout<<foot_end_force<<std::endl;

    return foot_end_force;
}

void ROSLegTopicHandle::legGazeboSimInit(int argc, char *argv[])
{
    ros::init(argc,argv,"LegPositionPubInit");
    ros::NodeHandle ROSLegTopicHandle;

    lfLegPub[0]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/L1_1_jointcc/command",1000);
    lfLegPub[1]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/L1_2_jointcc/command",1000);
    lfLegPub[2]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/L1_3_jointcc/command",1000);

    lmLegPub[0]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/L2_1_jointcc/command",1000);
    lmLegPub[1]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/L2_2_jointcc/command",1000);
    lmLegPub[2]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/L2_3_jointcc/command",1000);

    lbLegPub[0]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/L3_1_jointcc/command",1000);
    lbLegPub[1]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/L3_2_jointcc/command",1000);
    lbLegPub[2]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/L3_3_jointcc/command",1000);

    rfLegPub[0]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/R1_1_jointcc/command",1000);
    rfLegPub[1]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/R1_2_jointcc/command",1000);
    rfLegPub[2]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/R1_3_jointcc/command",1000);

    rmLegPub[0]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/R2_1_jointcc/command",1000);
    rmLegPub[1]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/R2_2_jointcc/command",1000);
    rmLegPub[2]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/R2_3_jointcc/command",1000);

    rbLegPub[0]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/R3_1_jointcc/command",1000);
    rbLegPub[1]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/R3_2_jointcc/command",1000);
    rbLegPub[2]=ROSLegTopicHandle.advertise<std_msgs::Float64>("/newtwo/R3_3_jointcc/command",1000);

    yobotics_imu_msg = ROSLegTopicHandle.subscribe<sensor_msgs::Imu>("/imu",1000,doMsg_yobotics_imu_msg);
    yobotics_joint_states = ROSLegTopicHandle.subscribe<sensor_msgs::JointState>("/newtwo/joint_states",1000,doMsg_yobotics_joint_states);
    yobotics_nav_odometry = ROSLegTopicHandle.subscribe<nav_msgs::Odometry>("/POS_COM",1000,doMsg_nav_odometry);

    bumper_contacts_sub_leg1 = ROSLegTopicHandle.subscribe<geometry_msgs::WrenchStamped>("/L1_4J_ft",1000,doMsg_bumper_L1_states);
    bumper_contacts_sub_leg2 = ROSLegTopicHandle.subscribe<geometry_msgs::WrenchStamped>("/L2_4J_ft",1000,doMsg_bumper_L2_states);
    bumper_contacts_sub_leg3 = ROSLegTopicHandle.subscribe<geometry_msgs::WrenchStamped>("/L3_4J_ft",1000,doMsg_bumper_L3_states);
    bumper_contacts_sub_leg4 = ROSLegTopicHandle.subscribe<geometry_msgs::WrenchStamped>("/R1_4J_ft",1000,doMsg_bumper_R1_states);
    bumper_contacts_sub_leg5 = ROSLegTopicHandle.subscribe<geometry_msgs::WrenchStamped>("/R2_4J_ft",1000,doMsg_bumper_R2_states);
    bumper_contacts_sub_leg6 = ROSLegTopicHandle.subscribe<geometry_msgs::WrenchStamped>("/R3_4J_ft",1000,doMsg_bumper_R3_states);
}

void ROSLegTopicHandle::legMsgSUb(void)
{   
    // ros::NodeHandle ROSLegTopicHandle;
    // yobotics_joint_states = ROSLegTopicHandle.subscribe<sensor_msgs::JointState>("/yobotics/joint_states",1000,doMsg_yobotics_joint_states);
}

void ROSLegTopicHandle::imuMsgSUb(void)
{   
    // ros::NodeHandle ROSImuTopicHandle;
    // yobotics_imu_msg = ROSImuTopicHandle.subscribe<sensor_msgs::Imu>("/imu",1000,doMsg_yobotics_imu_msg);
}


// void ROSLegTopicHandle::legMsgPub(double *lf_msg,double *lm_msg,double *lb_msg,
//                 double *rf_msg,double *rm_msg,double *rb_msg)
void ROSLegTopicHandle::legMsgPub(Eigen::Matrix<double,3,6> pub_msg)
{
    Eigen::Vector3d lfMsgTemp,lmMsgTemp,lbMsgTemp,rfMsgTemp,rmMsgTemp,rbMsgTemp;

    lfMsgTemp=pub_msg.block<3,1>(0,0);
    lmMsgTemp=pub_msg.block<3,1>(0,1);
    lbMsgTemp=pub_msg.block<3,1>(0,2);
    rfMsgTemp=pub_msg.block<3,1>(0,3);
    rmMsgTemp=pub_msg.block<3,1>(0,4);
    rbMsgTemp=pub_msg.block<3,1>(0,5);

    if(lfMsgProte.StopFlag==false or lmMsgProte.StopFlag==false or lbMsgProte.StopFlag==false
    or rfMsgProte.StopFlag==false or rmMsgProte.StopFlag==false or rbMsgProte.StopFlag==false)
    {
        lfMsgProte.StopFlag=false;lmMsgProte.StopFlag=false;lbMsgProte.StopFlag=false;
        rfMsgProte.StopFlag=false;rmMsgProte.StopFlag=false;rbMsgProte.StopFlag=false;
    }

    lfTempMsg[0].data=lfMsgTemp(0); lfTempMsg[1].data=lfMsgTemp(1);    lfTempMsg[2].data=lfMsgTemp(2);   
    lfLegPub[0].publish(lfTempMsg[0]);lfLegPub[1].publish(lfTempMsg[1]);lfLegPub[2].publish(lfTempMsg[2]); 

    // // printf("lfTempMsg: %f %f %f \n", lfTempMsg[0].data*_RAD1, lfTempMsg[1].data*_RAD1, lfTempMsg[2].data*_RAD1);

    lmTempMsg[0].data=lmMsgTemp(0);  lmTempMsg[1].data=lmMsgTemp(1);    lmTempMsg[2].data=lmMsgTemp(2);   
    lmLegPub[0].publish(lmTempMsg[0]);lmLegPub[1].publish(lmTempMsg[1]);lmLegPub[2].publish(lmTempMsg[2]); 

    // // // printf("lmTempMsg: %f %f %f \n", lmTempMsg[0].data*_RAD1, lmTempMsg[1].data*_RAD1, lmTempMsg[2].data*_RAD1);

    lbTempMsg[0].data=lbMsgTemp(0);  lbTempMsg[1].data=lbMsgTemp(1);    lbTempMsg[2].data=lbMsgTemp(2);   
    lbLegPub[0].publish(lbTempMsg[0]);lbLegPub[1].publish(lbTempMsg[1]);lbLegPub[2].publish(lbTempMsg[2]); 

    // // // printf("lbTempMsg: %f %f %f \n", lbTempMsg[0].data*_RAD1, lbTempMsg[1].data*_RAD1, lbTempMsg[2].data*_RAD1);

    rfTempMsg[0].data=-rfMsgTemp(0);  rfTempMsg[1].data=-rfMsgTemp(1);    rfTempMsg[2].data=-rfMsgTemp(2);   
    rfLegPub[0].publish(rfTempMsg[0]);rfLegPub[1].publish(rfTempMsg[1]);rfLegPub[2].publish(rfTempMsg[2]); 

    // // // printf("rfTempMsg: %f %f %f \n", rfTempMsg[0].data*_RAD1, rfTempMsg[1].data*_RAD1, rfTempMsg[2].data*_RAD1);

    rmTempMsg[0].data=-rmMsgTemp(0);  rmTempMsg[1].data=-rmMsgTemp(1);    rmTempMsg[2].data=-rmMsgTemp(2);   
    rmLegPub[0].publish(rmTempMsg[0]);rmLegPub[1].publish(rmTempMsg[1]);rmLegPub[2].publish(rmTempMsg[2]); 

    // // // printf("rmTempMsg: %f %f %f \n", rmTempMsg[0].data*_RAD1, rmTempMsg[1].data*_RAD1, rmTempMsg[2].data*_RAD1);

    rbTempMsg[0].data=-rbMsgTemp(0);  rbTempMsg[1].data=-rbMsgTemp(1);    rbTempMsg[2].data=-rbMsgTemp(2);   
    rbLegPub[0].publish(rbTempMsg[0]); rbLegPub[1].publish(rbTempMsg[1]);rbLegPub[2].publish(rbTempMsg[2]); 

    // // printf("rbTempMsg: %f %f %f \n", rbTempMsg[0].data*_RAD1, rbTempMsg[1].data*_RAD1, rbTempMsg[2].data*_RAD1);
}
