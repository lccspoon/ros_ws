/*! @file SimulationBridge.h
*  @brief  The SimulationBridge runs a RobotController and connects it to a
* Simulator, using shared memory. It is the simulation version of the
* HardwareBridge.
*/
#ifndef SIMULATIONBRIDGE_H
#define SIMULATIONBRIDGE_H
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include <iostream>
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/ContactsState.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/WrenchStamped.h" 
#include "../../common/utilities/protection.h"
#include <mutex>

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
using namespace std;

extern int SIM_SEQ;

class ROSLegTopicHandle
{
    protected:
        // std::mutex mux_joint;
        ros::Publisher lfLegPub[3],lmLegPub[3],lbLegPub[3],rfLegPub[3],rmLegPub[3],rbLegPub[3];
        ros::Subscriber yobotics_joint_states, yobotics_imu_msg, yobotics_nav_odometry;
        ros::Subscriber  bumper_contacts_sub_leg1, bumper_contacts_sub_leg2, bumper_contacts_sub_leg3;
        ros::Subscriber  bumper_contacts_sub_leg4, bumper_contacts_sub_leg5, bumper_contacts_sub_leg6;
        std_msgs::Float64 lfTempMsg[3],lmTempMsg[3],lbTempMsg[3],rfTempMsg[3],rmTempMsg[3],rbTempMsg[3];
        double _RAD1=180/3.1415;

        DataUnusualProtect lfMsgProte,lmMsgProte,lbMsgProte,rfMsgProte,rmMsgProte,rbMsgProte;

        Eigen::Matrix<double,3,6> foot_end_force; 
        Eigen::Matrix<double,3,6> foot_end_torque; 



    public:

        int seq=0,seq_last=0;
        double * position,* velocity,* effort;
        void legGazeboSimInit(int argc, char *argv[]);
        void legMsgPub(Eigen::Matrix<double,3,6> pub_msg);
        void legMsgSUb(void);
        Eigen::Matrix<double,18,1> retLegMsgPosition(void);
        Eigen::Matrix<double,18,1> retLegMsgVelocity(void);
        Eigen::Matrix<double,18,1> retLegMsgTorque(void);
        int retLegMsgSeq(void);

        void imuMsgSUb(void);
        Eigen::Matrix<double,4,1>  retImuQuaOri(void);
        Eigen::Matrix<double,3,1> retImuAngVel(void);
        Eigen::Matrix<double,3,1>  retImuLinAcc(void);
        int retImuSeq(void);

        double * retOdoPostion(void);
        double * retOdoTwistLinear(void);
        int retOdoSeq(void);

        Eigen::Matrix<double,3,6> retFootEndForec(void);    

};

class pubMsgTopicName
{
    protected:
        ros::Publisher msgPub;
        std_msgs::Float64 msgTemp;
        std_msgs::Float64MultiArray msgTempArray;
        ros::NodeHandle node;

    public:

    pubMsgTopicName(string  topicName)  
    {
        msgPub=node.advertise<std_msgs::Float64MultiArray>(topicName,1000);
        cout<<topicName<<endl;
    }

    void msgPubRun(double * msg_array)
    {
        msgTempArray.data={msg_array[0],msg_array[1],msg_array[2]};
        msgPub.publish(msgTempArray);
    }
    
    void msgPubRun(Eigen::Matrix<double,3,1> msg_matrix)
    {
        msgTempArray.data={msg_matrix(0),msg_matrix(1),msg_matrix(2)};
        msgPub.publish(msgTempArray);
    }

    // void msgPubRun(Vector3d msg_matrix)
    // {
    //     msgTempArray.data={msg_matrix(0),msg_matrix(1),msg_matrix(2)};
    //     msgPub.publish(msgTempArray);
    // }

    void msgPubRun(Eigen::Matrix<double,4,1> msg_matrix)
    {
        msgTempArray.data={msg_matrix(0),msg_matrix(1),msg_matrix(2),msg_matrix(3)};
        msgPub.publish(msgTempArray);
    }

    void msgPubRun(Eigen::Matrix<double,6,1> msg_matrix)
    {
        msgTempArray.data={msg_matrix(0),msg_matrix(1),msg_matrix(2),msg_matrix(3),msg_matrix(4),msg_matrix(5)};
        msgPub.publish(msgTempArray);
    }

    void msgPubRun(Eigen::Matrix<double,18,1> msg_matrix)
    {
        msgTempArray.data={

            msg_matrix(0),msg_matrix(1),msg_matrix(2),msg_matrix(3),msg_matrix(4),msg_matrix(5),
            msg_matrix(6),msg_matrix(7),msg_matrix(8),msg_matrix(9),msg_matrix(10),msg_matrix(11),
            msg_matrix(12),msg_matrix(13),msg_matrix(14),msg_matrix(15),msg_matrix(16),msg_matrix(17),
            
            };
        msgPub.publish(msgTempArray);
    }

};



#endif
