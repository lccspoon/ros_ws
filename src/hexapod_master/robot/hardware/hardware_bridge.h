#ifndef _HARDWARE_BRIDGE_H
#define _HARDWARE_BRIDGE_H

#include "PiceCan.h"
#include "../../common/utilities/protection.h"
#include <Eigen/Core>
#include <Eigen/Dense>
class HardwareBridge
{
    private:
        double _RAD2=180/3.1415926;
        VCI_INIT_CONFIG config;//
        int _board_num;
        int _board_set_order[3];
        VCI_CAN_OBJ rectest[3000];
        int count;
        int _ret_real_board_equnum[3]={0};

        Eigen::Matrix<double,1,18> whole_pos_data, whole_vel_data, whole_tor_data;

        int seq_update_flag_array[3]={0};

        DataUnusualProtect _LfDatUnuProte,_LmDatUnuProte,_LbDatUnuProte,_RfDatUnuProte,_RmDatUnuProte,_RbDatUnuProte;
    public:
        double _RAD1=3.1415926/180;
        int output_sequnce=0;
        static int global_send_thread_flag;
        PiceCan CanDevice0,CanDevice1,CanDevice2,CanDevice3,CanDevice4,CanDevice5;

        Eigen::Vector3d lf_des_pos, lm_des_pos, lb_des_pos, rf_des_pos, rm_des_pos, rb_des_pos;
        Eigen::Vector3d lf_des_tor, lm_des_tor, lb_des_tor, rf_des_tor, rm_des_tor, rb_des_tor;
        Eigen::Vector3d lf_des_vel, lm_des_vel, lb_des_vel, rf_des_vel, rm_des_vel, rb_des_vel;
        Eigen::Vector3d lf_des_kp, lm_des_kp, lb_des_kp, rf_des_kp, rm_des_kp, rb_des_kp;
        Eigen::Vector3d lf_des_kd, lm_des_kd, lb_des_kd, rf_des_kd, rm_des_kd, rb_des_kd;



        Eigen::Vector3d lf_act_pos, lm_act_pos, lb_act_pos, rf_act_pos, rm_act_pos, rb_act_pos;
        Eigen::Vector3d lf_act_vel, lm_act_vel, lb_act_vel, rf_act_vel, rm_act_vel, rb_act_vel;
        Eigen::Vector3d lf_act_tor, lm_act_tor, lb_act_tor, rf_act_tor, rm_act_tor, rb_act_tor;

        int lf_can_dev_seq[3]={0},lm_can_dev_seq[3]={0},lb_can_dev_seq[3]={0},   
              rf_can_dev_seq[3]={0},rm_can_dev_seq[3]={0},rb_can_dev_seq[3]={0};
        int lf_can_dev_seq_last[3]={0},lm_can_dev_seq_last[3]={0},lb_can_dev_seq_last[3]={0},   
              rf_can_dev_seq_last[3]={0},rm_can_dev_seq_last[3]={0},rb_can_dev_seq_last[3]={0};

        void creatCanBoard0(int testnum0,int equ_number);
        void creatCanBoard1(int testnum1,int equ_number);
        void creatCanBoard2(int testnum2,int equ_number);
        int * boardOrder(void);
        int * boardEquNum(void);

        void dataReceiveAll(void); 

        void dataRecAllAndSeqUpdate(void);

        void dataIntegrat(void);
        Eigen::Matrix<double,1,18> retWhloePos(void);

        Eigen::Matrix<double,1,18> retWhloeVel(void);

        Eigen::Matrix<double,1,18> retWhloeTor(void);

        void lfDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_);
        void lmDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_);
        void lbDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_);
        void rfDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_);
        void rmDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_);
        void rbDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_);

        void robDataLoadExitCloesLoop();

        void robDataLoadEnterCloseLoop();

        void robDataLoadEnterCloseLoop_lf();
        void robDataLoadEnterCloseLoop_lm();
        void robDataLoadEnterCloseLoop_lb();
        void robDataLoadEnterCloseLoop_rf();
        void robDataLoadEnterCloseLoop_rm();
        void robDataLoadEnterCloseLoop_rb();


        void robDataLoadSetZeroPoint();

        void showAllData(void);

        void sendAllLegMsg(void);

        void openSendThread(int key);


        DataUnusualProtect LfProtect,LmProtect,LbProtect,RfProtect,RmProtect,RbProtect;
        int lf_protect_init_flag=0, lm_protect_init_flag=0, lb_protect_init_flag=0,
            rf_protect_init_flag=0, rm_protect_init_flag=0, rb_protect_init_flag=0;


        // float lf_off_set[3]={0}, lm_off_set[3]={0}, lb_off_set[3]={0} ,
        //         rf_off_set[3]={0}, rm_off_set[3]={0}, rb_off_set[3]={0};
        Eigen::Vector3d lf_off_set, lm_off_set, lb_off_set, rf_off_set, rm_off_set, rb_off_set;
};

extern HardwareBridge HarBri;
# endif