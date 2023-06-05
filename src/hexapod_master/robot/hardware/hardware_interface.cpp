/**
* @file hardware_interface.h
* @brief 用来定义，声明在hardware_bridge.h中
* @details 定义了硬件与hexapod数据交互等接口。
* @author lcc
* @date date at : 20230330
*/
#include "hardware_interface.h"
void HardwareBridge::dataReceiveAll(void)
{
    // HarBri.lf_off_set[0]=3.573610*HarBri._RAD1;
    // HarBri.lf_off_set[1]=0.185784*HarBri._RAD1;
    // HarBri.lf_off_set[2]=2.409888*HarBri._RAD1;

    // HarBri.lm_off_set[0]=5.606305*HarBri._RAD1;
    // HarBri.lm_off_set[1]=1.672056*HarBri._RAD1;
    // HarBri.lm_off_set[2]=-0.161596*HarBri._RAD1;

    // HarBri.lb_off_set[0]=2.196622*HarBri._RAD1;
    // HarBri.lb_off_set[1]=1.366059*HarBri._RAD1;
    // HarBri.lb_off_set[2]=2.185059*HarBri._RAD1;

    // HarBri.rf_off_set[0]=-0.775921*HarBri._RAD1;
    // HarBri.rf_off_set[1]=-1.584628*HarBri._RAD1;
    // HarBri.rf_off_set[2]=-0.442633*HarBri._RAD1;

    // HarBri.rm_off_set[0]=-0.469924*HarBri._RAD1;
    // HarBri.rm_off_set[1]=-3.114614*HarBri._RAD1;
    // HarBri.rm_off_set[2]=2.128852*HarBri._RAD1;

    // HarBri.rb_off_set[0]=2.874188*HarBri._RAD1;
    // HarBri.rb_off_set[1]=-2.874188*HarBri._RAD1;
    // HarBri.rb_off_set[2]=1.341949*HarBri._RAD1;
    double  pos[3], vel[3], tor[3];
    CanDevice0.mtx.lock();
    for(int i=0;i<3;i++)
    {
        // CanDevice0.mtx_test_data++;
        // std::cout<<"----------------------dataReceiveAll mtx_test_data:"<<CanDevice0.mtx_test_data<<std::endl;
        pos[i]=CanDevice0.ret_pos[i];
        vel[i]=CanDevice0.ret_vel[i];
        tor[i]=CanDevice0.ret_tor[i];rf_can_dev_seq[i]=CanDevice0.ret_seq[i];
        rf_act_pos(i)=pos[i];
        rf_act_vel(i)=vel[i];
        rf_act_tor(i)=tor[i];
    }
    // rf_act_pos[0]=CanDevice0.ret_pos[0]-HarBri.rf_off_set[0];
    // rf_act_pos[1]=CanDevice0.ret_pos[1]-HarBri.rf_off_set[1];
    // rf_act_pos[2]=CanDevice0.ret_pos[2]-HarBri.rf_off_set[2];
    CanDevice0.mtx.unlock();

    CanDevice1.mtx.lock();
    for(int i=0;i<3;i++)
    {
        pos[i]=CanDevice1.ret_pos[i];
        vel[i]=CanDevice1.ret_vel[i];
        tor[i]=CanDevice1.ret_tor[i];lf_can_dev_seq[i]=CanDevice1.ret_seq[i];
        lf_act_pos(i)=pos[i];
        lf_act_vel(i)=vel[i];
        lf_act_tor(i)=tor[i];
    }
    // lf_act_pos[0]=CanDevice0.ret_pos[0]-HarBri.lf_off_set[0];
    // lf_act_pos[1]=CanDevice0.ret_pos[1]-HarBri.lf_off_set[1];
    // lf_act_pos[2]=CanDevice0.ret_pos[2]-HarBri.lf_off_set[2];
    CanDevice1.mtx.unlock();

    CanDevice4.mtx.lock();
    for(int i=0;i<3;i++)
    {
        pos[i]=CanDevice4.ret_pos[i];
        vel[i]=CanDevice4.ret_vel[i];
        tor[i]=CanDevice4.ret_tor[i];rb_can_dev_seq[i]=CanDevice4.ret_seq[i];
        rb_act_pos(i)=pos[i];
        rb_act_vel(i)=vel[i];
        rb_act_tor(i)=tor[i];
    }
    // rb_act_pos[0]=CanDevice0.ret_pos[0]-HarBri.rb_off_set[0];
    // rb_act_pos[1]=CanDevice0.ret_pos[1]-HarBri.rb_off_set[1];
    // rb_act_pos[2]=CanDevice0.ret_pos[2]-HarBri.rb_off_set[2];
    CanDevice4.mtx.unlock();

    CanDevice5.mtx.lock();
    for(int i=0;i<3;i++)
    {
        pos[i]=CanDevice5.ret_pos[i];
        vel[i]=CanDevice5.ret_vel[i];
        tor[i]=CanDevice5.ret_tor[i];lb_can_dev_seq[i]=CanDevice5.ret_seq[i];
        lb_act_pos(i)=pos[i];
        lb_act_vel(i)=vel[i];
        lb_act_tor(i)=tor[i];
    }
    // lb_act_pos[0]=CanDevice0.ret_pos[0]-HarBri.lb_off_set[0];
    // lb_act_pos[1]=CanDevice0.ret_pos[1]-HarBri.lb_off_set[1];
    // lb_act_pos[2]=CanDevice0.ret_pos[2]-HarBri.lb_off_set[2];
    CanDevice5.mtx.unlock();

    CanDevice3.mtx.lock();
    for(int i=0;i<3;i++)
    {
        pos[i]=CanDevice3.ret_pos[i];
        vel[i]=CanDevice3.ret_vel[i];
        tor[i]=CanDevice3.ret_tor[i];rm_can_dev_seq[i]=CanDevice3.ret_seq[i];
        rm_act_pos(i)=pos[i];
        rm_act_vel(i)=vel[i];
        rm_act_tor(i)=tor[i];
    }
    // rm_act_pos[0]=CanDevice0.ret_pos[0]-HarBri.rm_off_set[0];
    // rm_act_pos[1]=CanDevice0.ret_pos[1]-HarBri.rm_off_set[1];
    // rm_act_pos[2]=CanDevice0.ret_pos[2]-HarBri.rm_off_set[2];
    CanDevice3.mtx.unlock();

    CanDevice2.mtx.lock();
    for(int i=0;i<3;i++)
    {
        pos[i]=CanDevice2.ret_pos[i];
        vel[i]=CanDevice2.ret_vel[i];
        tor[i]=CanDevice2.ret_tor[i];lm_can_dev_seq[i]=CanDevice2.ret_seq[i];
        lm_act_pos(i)=pos[i];
        lm_act_vel(i)=vel[i];
        lm_act_tor(i)=tor[i];
    }
    // lm_act_pos[0]=CanDevice0.ret_pos[0]-HarBri.lm_off_set[0];
    // lm_act_pos[1]=CanDevice0.ret_pos[1]-HarBri.lm_off_set[1];
    // lm_act_pos[2]=CanDevice0.ret_pos[2]-HarBri.lm_off_set[2];
    CanDevice2.mtx.unlock();
}       
std::mutex mtx5;
void HardwareBridge::dataRecAllAndSeqUpdate(void)
{
    dataReceiveAll();
    mtx5.lock();
    for(int i=0;i<3;i++)
    {
        if( 
            lf_can_dev_seq[i]==lf_can_dev_seq_last[i] ||
            lm_can_dev_seq[i]==lm_can_dev_seq_last[i] ||
            lb_can_dev_seq[i]==lb_can_dev_seq_last[i] ||

            rf_can_dev_seq[i]==rf_can_dev_seq_last[i] ||
            rm_can_dev_seq[i]==rm_can_dev_seq_last[i] ||
            rb_can_dev_seq[i]==rb_can_dev_seq_last[i] 
            )
        {
            seq_update_flag_array[i]=0;
            // printf("seq_update_flag_array[%d]:%d\n",i,seq_update_flag_array[i]);
        }
        else
        {
            seq_update_flag_array[i]=1;
            // printf("seq_update_flag_array[%d]:%d\n",i,seq_update_flag_array[i]);
        }
    }

    if(seq_update_flag_array[0]==1 && seq_update_flag_array[1]==1 && seq_update_flag_array[2]==1)
    {
        // global_send_thread_flag=1;  // 1 is represent to ok!  let send Thread continue------
        output_sequnce++;
        for(int i=0;i<3;i++)
        {
            lf_can_dev_seq_last[i]=lf_can_dev_seq[i];
            lm_can_dev_seq_last[i]=lm_can_dev_seq[i];
            lb_can_dev_seq_last[i]=lb_can_dev_seq[i];
            rf_can_dev_seq_last[i]=rf_can_dev_seq[i];
            rm_can_dev_seq_last[i]=rm_can_dev_seq[i];
            rb_can_dev_seq_last[i]=rb_can_dev_seq[i];
        }
    }
    else
    {
        // global_send_thread_flag=0;
    }

    // std::cout<<"global_send_thread_flag:"<<global_send_thread_flag<<std::endl;
    // std::cout<<"output_sequnce:"<<output_sequnce<<std::endl;
    mtx5.unlock();

}

void HardwareBridge::dataIntegrat(void)
{
    whole_pos_data(0)=lf_act_pos(0);whole_pos_data(1)=lf_act_pos(1);whole_pos_data(2)=lf_act_pos(2);
    whole_pos_data(3)=lm_act_pos(0);whole_pos_data(4)=lm_act_pos(1);whole_pos_data(5)=lm_act_pos(2);
    whole_pos_data(6)=lb_act_pos(0);whole_pos_data(7)=lb_act_pos(1);whole_pos_data(8)=lb_act_pos(2);
    whole_pos_data(9)=rf_act_pos(0);whole_pos_data(10)=rf_act_pos(1);whole_pos_data(11)=rf_act_pos(2);
    whole_pos_data(12)=rm_act_pos(0);whole_pos_data(13)=rm_act_pos(1);whole_pos_data(14)=rm_act_pos(2);
    whole_pos_data(15)=rb_act_pos(0);whole_pos_data(16)=rb_act_pos(1);whole_pos_data(17)=rb_act_pos(2);

    whole_vel_data(0)=lf_act_vel(0);whole_vel_data(1)=lf_act_vel(1);whole_vel_data(2)=lf_act_vel(2);
    whole_vel_data(3)=lm_act_vel(0);whole_vel_data(4)=lm_act_vel(1);whole_vel_data(5)=lm_act_vel(2);
    whole_vel_data(6)=lb_act_pos(0);whole_vel_data(7)=lb_act_pos(1);whole_vel_data(8)=lb_act_pos(2);
    whole_vel_data(9)=rf_act_vel(0);whole_vel_data(10)=rf_act_vel(1);whole_vel_data(11)=rf_act_vel(2);
    whole_vel_data(12)=rm_act_pos(0);whole_vel_data(13)=rm_act_pos(1);whole_vel_data(14)=rm_act_pos(2);
    whole_vel_data(15)=rb_act_pos(0);whole_vel_data(16)=rb_act_pos(1);whole_vel_data(17)=rb_act_pos(2);

    whole_tor_data(0)=lf_act_tor(0);whole_tor_data(1)=lf_act_tor(1);whole_tor_data(2)=lf_act_tor(2);
    whole_tor_data(3)=lm_act_tor(0);whole_tor_data(4)=lm_act_tor(1);whole_tor_data(5)=lm_act_tor(2);
    whole_tor_data(6)=lb_act_tor(0);whole_tor_data(7)=lb_act_tor(1);whole_tor_data(8)=lb_act_tor(2);
    whole_tor_data(9)=rf_act_tor(0);whole_tor_data(10)=rf_act_tor(1);whole_tor_data(11)=rf_act_tor(2);
    whole_tor_data(12)=rm_act_tor(0);whole_tor_data(13)=rm_act_tor(1);whole_tor_data(14)=rm_act_tor(2);
    whole_tor_data(15)=rb_act_tor(0);whole_tor_data(16)=rb_act_tor(1);whole_tor_data(17)=rb_act_tor(2);
}

Eigen::Matrix<double,1,18> HardwareBridge::retWhloePos(void)
{
    return whole_pos_data;
}

Eigen::Matrix<double,1,18> HardwareBridge::retWhloeVel(void)
{
    return whole_vel_data;
}

Eigen::Matrix<double,1,18> HardwareBridge::retWhloeTor(void)
{
    return whole_tor_data;
}

std::mutex mtx_1;
std::mutex mtx_2;
std::mutex mtx_3;
std::mutex mtx_4;
std::mutex mtx_5;
std::mutex mtx_6;
// void HardwareBridge::lfDataLoading(double * pos,double * vel,double * trop,double * KP,double * KD)
void HardwareBridge::lfDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_)
{   
    mtx_1.lock();
        double pos[3], vel[3], trop[3], KP[3], KD[3];
        for (int i = 0; i < 3; i++)
        {
            pos[i]=pos_(i);
            vel[i]=vel_(i);
            trop[i]=trop_(i);
            KP[i]=KP_(i);
            KD[i]=KD_(i);
        }
        CanDevice1.loaSenMsgData1(pos[0],vel[0],trop[0],KP[0],KD[0]);    
        CanDevice1.loaSenMsgData2(pos[1],vel[1],trop[1],KP[1],KD[1]);    
        CanDevice1.loaSenMsgData3(pos[2],vel[2],trop[2],KP[2],KD[2]);    
    mtx_1.unlock();
}

void HardwareBridge::lmDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_)
{
    mtx_2.lock();
        double pos[3], vel[3], trop[3], KP[3], KD[3];
        for (int i = 0; i < 3; i++)
        {
            pos[i]=pos_(i);
            vel[i]=vel_(i);
            trop[i]=trop_(i);
            KP[i]=KP_(i);
            KD[i]=KD_(i);
        }
    // CanDevice2.loaSenMsgData(pos,vel,trop,KP,KD);   //lm 2
    CanDevice2.loaSenMsgData1(pos[0],vel[0],trop[0],KP[0],KD[0]);    
    CanDevice2.loaSenMsgData2(pos[1],vel[1],trop[1],KP[1],KD[1]);    
    CanDevice2.loaSenMsgData3(pos[2],vel[2],trop[2],KP[2],KD[2]);    
    mtx_2.unlock();
}

void HardwareBridge::lbDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_)
{
    mtx_3.lock();
        double pos[3], vel[3], trop[3], KP[3], KD[3];
        for (int i = 0; i < 3; i++)
        {
            pos[i]=pos_(i);
            vel[i]=vel_(i);
            trop[i]=trop_(i);
            KP[i]=KP_(i);
            KD[i]=KD_(i);
        }
    // CanDevice5.loaSenMsgData(pos,vel,trop,KP,KD);   //lb 5
    CanDevice5.loaSenMsgData1(pos[0],vel[0],trop[0],KP[0],KD[0]);    
    CanDevice5.loaSenMsgData2(pos[1],vel[1],trop[1],KP[1],KD[1]);    
    CanDevice5.loaSenMsgData3(pos[2],vel[2],trop[2],KP[2],KD[2]);    
    mtx_3.unlock();
}

void HardwareBridge::rfDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_)
{
    mtx_4.lock();
        double pos[3], vel[3], trop[3], KP[3], KD[3];
        for (int i = 0; i < 3; i++)
        {
            pos[i]=pos_(i);
            vel[i]=vel_(i);
            trop[i]=trop_(i);
            KP[i]=KP_(i);
            KD[i]=KD_(i);
        }
    // CanDevice0.loaSenMsgData(pos,vel,trop,KP,KD);   //rf 0
    CanDevice0.loaSenMsgData1(pos[0],vel[0],trop[0],KP[0],KD[0]);    
    CanDevice0.loaSenMsgData2(pos[1],vel[1],trop[1],KP[1],KD[1]);    
    CanDevice0.loaSenMsgData3(pos[2],vel[2],trop[2],KP[2],KD[2]);    
    mtx_4.unlock();
}

void HardwareBridge::rmDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_)
{
    mtx_5.lock();
        double pos[3], vel[3], trop[3], KP[3], KD[3];
        for (int i = 0; i < 3; i++)
        {
            pos[i]=pos_(i);
            vel[i]=vel_(i);
            trop[i]=trop_(i);
            KP[i]=KP_(i);
            KD[i]=KD_(i);
        }
    // CanDevice3.loaSenMsgData(pos,vel,trop,KP,KD);    //rm 3
    CanDevice3.loaSenMsgData1(pos[0],vel[0],trop[0],KP[0],KD[0]);    
    CanDevice3.loaSenMsgData2(pos[1],vel[1],trop[1],KP[1],KD[1]);    
    CanDevice3.loaSenMsgData3(pos[2],vel[2],trop[2],KP[2],KD[2]);    
    mtx_5.unlock();
}

void HardwareBridge::rbDataLoading(Eigen::Vector3d pos_,Eigen::Vector3d vel_,Eigen::Vector3d trop_,Eigen::Vector3d KP_,Eigen::Vector3d KD_)
{
    mtx_6.lock();
        double pos[3], vel[3], trop[3], KP[3], KD[3];
        for (int i = 0; i < 3; i++)
        {
            pos[i]=pos_(i);
            vel[i]=vel_(i);
            trop[i]=trop_(i);
            KP[i]=KP_(i);
            KD[i]=KD_(i);
        }
    // CanDevice4.loaSenMsgData(pos,vel,trop,KP,KD);   //rb 4
    CanDevice4.loaSenMsgData1(pos[0],vel[0],trop[0],KP[0],KD[0]);    
    CanDevice4.loaSenMsgData2(pos[1],vel[1],trop[1],KP[1],KD[1]);    
    CanDevice4.loaSenMsgData3(pos[2],vel[2],trop[2],KP[2],KD[2]);    
    mtx_6.unlock();
}

void HardwareBridge::robDataLoadExitCloesLoop()
{
    CanDevice0.loadSenMsgExiCloLoo();
    CanDevice1.loadSenMsgExiCloLoo();
    CanDevice2.loadSenMsgExiCloLoo();
    CanDevice3.loadSenMsgExiCloLoo();
    CanDevice4.loadSenMsgExiCloLoo();
    CanDevice5.loadSenMsgExiCloLoo();
}

void HardwareBridge::robDataLoadEnterCloseLoop()
{
    CanDevice0.loadSenMsgEntCloLoo();
    CanDevice1.loadSenMsgEntCloLoo();
    CanDevice2.loadSenMsgEntCloLoo();
    CanDevice3.loadSenMsgEntCloLoo();
    CanDevice4.loadSenMsgEntCloLoo();
    CanDevice5.loadSenMsgEntCloLoo();  //loadSenMsgSetZeroPoint
}

void HardwareBridge::robDataLoadEnterCloseLoop_lf()
{
    CanDevice1.loadSenMsgEntCloLoo();
}
void HardwareBridge::robDataLoadEnterCloseLoop_rf()
{
    CanDevice0.loadSenMsgEntCloLoo();
}
void HardwareBridge::robDataLoadEnterCloseLoop_lm()
{
    CanDevice2.loadSenMsgEntCloLoo();
}
void HardwareBridge::robDataLoadEnterCloseLoop_rm()
{
    CanDevice3.loadSenMsgEntCloLoo();
}
void HardwareBridge::robDataLoadEnterCloseLoop_lb()
{
    CanDevice5.loadSenMsgEntCloLoo();
}
void HardwareBridge::robDataLoadEnterCloseLoop_rb()
{
    CanDevice4.loadSenMsgEntCloLoo();
}

void HardwareBridge::robDataLoadSetZeroPoint()
{
    // CanDevice0.loadSenMsgSetZeroPoint();
    // CanDevice1.loadSenMsgSetZeroPoint();
    // CanDevice2.loadSenMsgSetZeroPoint();
    // CanDevice3.loadSenMsgSetZeroPoint();
    // CanDevice4.loadSenMsgSetZeroPoint();
    // CanDevice5.loadSenMsgSetZeroPoint();  
}
std::mutex mtx9;
void HardwareBridge::showAllData(void)
{   mtx9.lock();

    printf("output_sequnce:%d \n",output_sequnce); printf("\n");

    printf(" lf_seq: ");     for(int i=0;i<3;i++)  printf(" %d ",lf_can_dev_seq[i]);    printf("\n");
    printf(" lm_seq: ");     for(int i=0;i<3;i++)  printf(" %d ",lm_can_dev_seq[i]);    printf("\n");
    printf(" lb_seq: ");     for(int i=0;i<3;i++)  printf(" %d ",lb_can_dev_seq[i]);    printf("\n");
    printf(" rf_seq: ");     for(int i=0;i<3;i++)  printf(" %d ",rf_can_dev_seq[i]);    printf("\n");
    printf(" rm_seq: ");     for(int i=0;i<3;i++)  printf(" %d ",rm_can_dev_seq[i]);    printf("\n");
    printf(" rb_seq: ");     for(int i=0;i<3;i++)  printf(" %d ",rb_can_dev_seq[i]);    printf("\n"); printf("\n");

    printf("send lf_des_pos: ");     for(int i=0;i<3;i++)  printf(" %5f ",lf_des_pos[i]*_RAD2);    printf("\n");
    printf("send lm_des_pos: ");     for(int i=0;i<3;i++)  printf(" %5f ",lm_des_pos[i]*_RAD2);    printf("\n");
    printf("send lb_des_pos: ");     for(int i=0;i<3;i++)  printf(" %5f ",lb_des_pos[i]*_RAD2);    printf("\n");
    printf("send rf_des_pos: ");     for(int i=0;i<3;i++)  printf(" %5f ",rf_des_pos[i]*_RAD2);    printf("\n");
    printf("send rm_des_pos: ");     for(int i=0;i<3;i++)  printf(" %5f ",rm_des_pos[i]*_RAD2);    printf("\n");
    printf("send rb_des_pos: ");     for(int i=0;i<3;i++)  printf(" %5f ",rb_des_pos[i]*_RAD2);    printf("\n"); printf("\n");

    printf("rec lf_act_pos: ");     for(int i=0;i<3;i++)  printf(" %5f ",whole_pos_data[i]*_RAD2);    printf("\n");
    printf("rec lm_act_pos: ");     for(int i=3;i<6;i++)  printf(" %5f ",whole_pos_data[i]*_RAD2);    printf("\n");
    printf("rec lb_act_pos: ");     for(int i=6;i<9;i++)  printf(" %5f ",whole_pos_data[i]*_RAD2);    printf("\n");
    printf("rec rf_act_pos: ");     for(int i=9;i<12;i++)  printf(" %5f ",whole_pos_data[i]*_RAD2);    printf("\n");
    printf("rec rm_act_pos: ");     for(int i=12;i<15;i++)  printf(" %5f ",whole_pos_data[i]*_RAD2);    printf("\n");
    printf("rec rb_act_pos: ");     for(int i=15;i<18;i++)  printf(" %5f ",whole_pos_data[i]*_RAD2);    printf("\n"); printf("\n");
    printf("\n");
    mtx9.unlock();

}

int sasas=100;
void HardwareBridge::sendAllLegMsg(void)
{
    CanDevice0.msgTransmit1();
    CanDevice1.msgTransmit1();
    CanDevice2.msgTransmit1();
    CanDevice3.msgTransmit1();
    CanDevice4.msgTransmit1();
    CanDevice5.msgTransmit1();
    usleep(sasas);

    CanDevice0.msgTransmit2();
    CanDevice1.msgTransmit2();
    CanDevice2.msgTransmit2();
    CanDevice3.msgTransmit2();
    CanDevice4.msgTransmit2();
    CanDevice5.msgTransmit2();
    usleep(sasas);

    CanDevice0.msgTransmit3();
    CanDevice1.msgTransmit3();
    CanDevice2.msgTransmit3();
    CanDevice3.msgTransmit3();
    CanDevice4.msgTransmit3();
    CanDevice5.msgTransmit3();
    usleep(sasas);

}



