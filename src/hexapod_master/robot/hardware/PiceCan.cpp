#include"PiceCan.h"
#include "time.h"

#define P_MIN  -12.5f;
#define P_MAX  12.5f;
#define V_MIN  -30.0f;
#define V_MAX  30.0f;
#define KP_MIN  0.0f;
#define KP_MAX  500.0f;
#define KD_MIN  0.0f;
#define KD_MAX  5.0f;
#define T_MIN  -10.0f;
#define T_MAX  10.0f;
int reccount=0,reccount1=0,reccount2=0,reccount3=0,reccount4=0,reccount6=0,reccount5=0;
int sendcount=0;

PiceCan::PiceCan(/* args */)
{
}

PiceCan::~PiceCan()
{
}

double PiceCan::uint_to_float(int x_int, double x_min, double x_max, int bits)
{
        double span = x_max - x_min;
        double offset = x_min;
        return ((double)x_int) * span / ((double)((1 << bits) - 1)) + offset;
}

int PiceCan::float_to_uint(double x, double x_min, double x_max, int bits)
{
        double span = x_max - x_min;
        double offset = x_min;
        return (int)((x - offset) * ((double)((1 << bits) - 1)) / span);
}

void PiceCan::createCanDev(int board,int dev_id,int motorId1,int motorId2,int motorId3)
{
    this->dev_id=dev_id;
    this->board=board;

	//需要发送的帧，结构体设置
	send[0].ID=motorId1;
	send[0].SendType=1;//1 only send once;0 continus 4s
	send[0].RemoteFlag=0;//0=standard; 1=remoteflag
	send[0].ExternFlag=0;//0=standard; 1=extend
	send[0].DataLen=8;//datalend
	send[0].TimeFlag=1;

    send1[0].ID=motorId2;
	send1[0].SendType=1;//1 only send once;0 continus 4s
	send1[0].RemoteFlag=0;//0=standard; 1=remoteflag
	send1[0].ExternFlag=0;//0=standard; 1=extend
	send1[0].DataLen=8;//datalend
	send1[0].TimeFlag=1;

    send2[0].ID=motorId3;
	send2[0].SendType=1;//1 only send once;0 continus 4s
	send2[0].RemoteFlag=0;//0=standard; 1=remoteflag
	send2[0].ExternFlag=0;//0=standard; 1=extend
	send2[0].DataLen=8;//datalend
	send2[0].TimeFlag=1;

}

void PiceCan::loadSenMsgEntCloLoo(void)
{
    send[0].Data[0] = 0xff;
    send[0].Data[1] = 0xff;
    send[0].Data[2] = 0xff;
    send[0].Data[3] = 0xff;
    send[0].Data[4] = 0xff; 
    send[0].Data[5] = 0xff;
    send[0].Data[6] = 0xff;
    send[0].Data[7] = 0xfc;

    send1[0].Data[0] = 0xff;
    send1[0].Data[1] = 0xff;
    send1[0].Data[2] = 0xff;
    send1[0].Data[3] = 0xff;
    send1[0].Data[4] = 0xff; 
    send1[0].Data[5] = 0xff;
    send1[0].Data[6] = 0xff;
    send1[0].Data[7] = 0xfc;

    send2[0].Data[0] = 0xff;
    send2[0].Data[1] = 0xff;
    send2[0].Data[2] = 0xff;
    send2[0].Data[3] = 0xff;
    send2[0].Data[4] = 0xff; 
    send2[0].Data[5] = 0xff;
    send2[0].Data[6] = 0xff;
    send2[0].Data[7] = 0xfc;
}

void PiceCan::loadSenMsgExiCloLoo(void)
{
    send[0].Data[0] = 0xff;
    send[0].Data[1] = 0xff;
    send[0].Data[2] = 0xff;
    send[0].Data[3] = 0xff;
    send[0].Data[4] = 0xff; 
    send[0].Data[5] = 0xff;
    send[0].Data[6] = 0xff;
    send[0].Data[7] = 0xfd;

    send1[0].Data[0] = 0xff;
    send1[0].Data[1] = 0xff;
    send1[0].Data[2] = 0xff;
    send1[0].Data[3] = 0xff;
    send1[0].Data[4] = 0xff; 
    send1[0].Data[5] = 0xff;
    send1[0].Data[6] = 0xff;
    send1[0].Data[7] = 0xfd;

    send2[0].Data[0] = 0xff;
    send2[0].Data[1] = 0xff;
    send2[0].Data[2] = 0xff;
    send2[0].Data[3] = 0xff;
    send2[0].Data[4] = 0xff; 
    send2[0].Data[5] = 0xff;
    send2[0].Data[6] = 0xff;
    send2[0].Data[7] = 0xfd;
}

void PiceCan::loadSenMsgSetZeroPoint(void)
{
    send[0].Data[0] = 0xff;
    send[0].Data[1] = 0xff;
    send[0].Data[2] = 0xff;
    send[0].Data[3] = 0xff;
    send[0].Data[4] = 0xff; 
    send[0].Data[5] = 0xff;
    send[0].Data[6] = 0xf1;
    send[0].Data[7] = 0x00;

    send1[0].Data[0] = 0xff;
    send1[0].Data[1] = 0xff;
    send1[0].Data[2] = 0xff;
    send1[0].Data[3] = 0xff;
    send1[0].Data[4] = 0xff; 
    send1[0].Data[5] = 0xff;
    send1[0].Data[6] = 0xf1;
    send1[0].Data[7] = 0x00;

    send2[0].Data[0] = 0xff;
    send2[0].Data[1] = 0xff;
    send2[0].Data[2] = 0xff;
    send2[0].Data[3] = 0xff;
    send2[0].Data[4] = 0xff; 
    send2[0].Data[5] = 0xff;
    send2[0].Data[6] = 0xf1;
    send2[0].Data[7] = 0x00;


    // send[0].Data[0] = 0x90;
    // send[0].Data[1] = 0x15;
    // send[0].Data[2] = 0x00;
    // send[0].Data[3] = 0x00;
    // send[0].Data[4] = 0x01; 
    // send[0].Data[5] = 0x00;
    // send[0].Data[6] = 0x07;
    // send[0].Data[7] = 0xff;

    // send1[0].Data[0] = 0x90;
    // send1[0].Data[1] = 0x15;
    // send1[0].Data[2] = 0x00;
    // send1[0].Data[3] = 0x00;
    // send1[0].Data[4] = 0x01; 
    // send1[0].Data[5] = 0x00;
    // send1[0].Data[6] = 0x07;
    // send1[0].Data[7] = 0xff;


    // send2[0].Data[0] = 0x90;
    // send2[0].Data[1] = 0x15;
    // send2[0].Data[2] = 0x00;
    // send2[0].Data[3] = 0x00;
    // send2[0].Data[4] = 0x01; 
    // send2[0].Data[5] = 0x00;
    // send2[0].Data[6] = 0x07;
    // send2[0].Data[7] = 0xff;
}

void PiceCan::loaSenMsgData(double * pos,double * vel,double * trop,double * KP,double * KD)
{
    mtx.lock();

        pos_tmp[0]=float_to_uint(pos[0],-12.5f,12.5f,16);
        vel_tmp[0]=float_to_uint(vel[0],-30.0f,30.0f,12);		
        kp_tmp[0]=float_to_uint(KP[0],0.0f,500.0f,12);
        kd_tmp[0]=float_to_uint(KD[0],0.0f,100.0f,12);
        tor_tmp[0]=float_to_uint(trop[0],-18.0f,18.0f,12); 

        pos_tmp[1]=float_to_uint(pos[1],-12.5f,12.5f,16);
        vel_tmp[1]=float_to_uint(vel[1],-30.0f,30.0f,12);		
        kp_tmp[1]=float_to_uint(KP[1],0.0f,500.0f,12);
        kd_tmp[1]=float_to_uint(KD[1],0.0f,100.0f,12);
        tor_tmp[1]=float_to_uint(trop[1],-18.0f,18.0f,12); 

        pos_tmp[2]=float_to_uint(pos[2],-12.5f,12.5f,16);
        vel_tmp[2]=float_to_uint(vel[2],-30.0f,30.0f,12);		
        kp_tmp[2]=float_to_uint(KP[2],0.0f,500.0f,12);
        kd_tmp[2]=float_to_uint(KD[2],0.0f,100.0f,12);
        tor_tmp[2]=float_to_uint(trop[2],-18.0f,18.0f,12); 

        send[0].Data[0] = (BYTE)(pos_tmp[0]>>8);
        send[0].Data[1] = (BYTE)(pos_tmp[0] & 0xFF);		
        send[0].Data[2] = (BYTE)((vel_tmp[0]>>4) & 0xFF);
        send[0].Data[3] = (BYTE)((((vel_tmp[0] & 0xF)<<4) & 0xFF)|((kp_tmp[0]>>8) & 0xFF));
        send[0].Data[4] = (BYTE)(kp_tmp[0] & 0xFF);
        send[0].Data[5] = (BYTE)((kd_tmp[0]>>4) & 0xFF);
        send[0].Data[6] = (BYTE)((((kd_tmp[0]&0xF)<<4) & 0xFF)|((tor_tmp[0]>>8) & 0xFF));
        send[0].Data[7] = (BYTE)(tor_tmp[0] & 0xFF);

        send1[0].Data[0] = (BYTE)(pos_tmp[1]>>8);
        send1[0].Data[1] = (BYTE)(pos_tmp[1] & 0xFF);		
        send1[0].Data[2] = (BYTE)((vel_tmp[1]>>4) & 0xFF);
        send1[0].Data[3] = (BYTE)((((vel_tmp[1] & 0xF)<<4) & 0xFF)|((kp_tmp[1]>>8) & 0xFF));
        send1[0].Data[4] = (BYTE)(kp_tmp[1] & 0xFF);
        send1[0].Data[5] = (BYTE)((kd_tmp[1]>>4) & 0xFF);
        send1[0].Data[6] = (BYTE)((((kd_tmp[1]&0xF)<<4) & 0xFF)|((tor_tmp[1]>>8) & 0xFF));
        send1[0].Data[7] = (BYTE)(tor_tmp[1] & 0xFF);

        send2[0].Data[0] = (BYTE)(pos_tmp[2]>>8);
        send2[0].Data[1] = (BYTE)(pos_tmp[2] & 0xFF);		
        send2[0].Data[2] = (BYTE)((vel_tmp[2]>>4) & 0xFF);
        send2[0].Data[3] = (BYTE)((((vel_tmp[2] & 0xF)<<4) & 0xFF)|((kp_tmp[2]>>8) & 0xFF));
        send2[0].Data[4] = (BYTE)(kp_tmp[2] & 0xFF);
        send2[0].Data[5] = (BYTE)((kd_tmp[2]>>4) & 0xFF);
        send2[0].Data[6] = (BYTE)((((kd_tmp[2]&0xF)<<4) & 0xFF)|((tor_tmp[2]>>8) & 0xFF));
        send2[0].Data[7] = (BYTE)(tor_tmp[2] & 0xFF);

    mtx.unlock();
    
}

void PiceCan::loaSenMsgData1(double pos,double vel,double trop,double  KP,double  KD)
{
    mtx.lock();

        pos_tmp[0]=float_to_uint(pos,-12.5f,12.5f,16);
        vel_tmp[0]=float_to_uint(vel,-30.0f,30.0f,12);		
        kp_tmp[0]=float_to_uint(KP,0.0f,500.0f,12);
        kd_tmp[0]=float_to_uint(KD,0.0f,5.0f,12);
        tor_tmp[0]=float_to_uint(trop,-10.0f,10.0f,12); 

        send[0].Data[0] = (BYTE)(pos_tmp[0]>>8);
        send[0].Data[1] = (BYTE)(pos_tmp[0] & 0xFF);		
        send[0].Data[2] = (BYTE)((vel_tmp[0]>>4) & 0xFF);
        send[0].Data[3] = (BYTE)((((vel_tmp[0] & 0xF)<<4) & 0xFF)|((kp_tmp[0]>>8) & 0xFF));
        send[0].Data[4] = (BYTE)(kp_tmp[0] & 0xFF);
        send[0].Data[5] = (BYTE)((kd_tmp[0]>>4) & 0xFF);
        send[0].Data[6] = (BYTE)((((kd_tmp[0]&0xF)<<4) & 0xFF)|((tor_tmp[0]>>8) & 0xFF));
        send[0].Data[7] = (BYTE)(tor_tmp[0] & 0xFF);

    mtx.unlock();
    
}
void PiceCan::loaSenMsgData2(double pos,double vel,double trop,double  KP,double  KD)
{
    mtx.lock();

        pos_tmp[0]=float_to_uint(pos,-12.5f,12.5f,16);
        vel_tmp[0]=float_to_uint(vel,-30.0f,30.0f,12);		
        kp_tmp[0]=float_to_uint(KP,0.0f,500.0f,12);
        kd_tmp[0]=float_to_uint(KD,0.0f,5.0f,12);
        tor_tmp[0]=float_to_uint(trop,-10.0f,10.0f,12); 

        send1[0].Data[0] = (BYTE)(pos_tmp[0]>>8);
        send1[0].Data[1] = (BYTE)(pos_tmp[0] & 0xFF);		
        send1[0].Data[2] = (BYTE)((vel_tmp[0]>>4) & 0xFF);
        send1[0].Data[3] = (BYTE)((((vel_tmp[0] & 0xF)<<4) & 0xFF)|((kp_tmp[0]>>8) & 0xFF));
        send1[0].Data[4] = (BYTE)(kp_tmp[0] & 0xFF);
        send1[0].Data[5] = (BYTE)((kd_tmp[0]>>4) & 0xFF);
        send1[0].Data[6] = (BYTE)((((kd_tmp[0]&0xF)<<4) & 0xFF)|((tor_tmp[0]>>8) & 0xFF));
        send1[0].Data[7] = (BYTE)(tor_tmp[0] & 0xFF);

    mtx.unlock();
    
}
void PiceCan::loaSenMsgData3(double pos,double vel,double trop,double  KP,double  KD)
{
    mtx.lock();
        pos=pos/knee_offset;
        pos_tmp[0]=float_to_uint(pos,-12.5f,12.5f,16);
        vel_tmp[0]=float_to_uint(vel,-30.0f,30.0f,12);		
        kp_tmp[0]=float_to_uint(KP,0.0f,500.0f,12);
        kd_tmp[0]=float_to_uint(KD,0.0f,5.0f,12);
        tor_tmp[0]=float_to_uint(trop,-10.0f,10.0f,12); 

        send2[0].Data[0] = (BYTE)(pos_tmp[0]>>8);
        send2[0].Data[1] = (BYTE)(pos_tmp[0] & 0xFF);		
        send2[0].Data[2] = (BYTE)((vel_tmp[0]>>4) & 0xFF);
        send2[0].Data[3] = (BYTE)((((vel_tmp[0] & 0xF)<<4) & 0xFF)|((kp_tmp[0]>>8) & 0xFF));
        send2[0].Data[4] = (BYTE)(kp_tmp[0] & 0xFF);
        send2[0].Data[5] = (BYTE)((kd_tmp[0]>>4) & 0xFF);
        send2[0].Data[6] = (BYTE)((((kd_tmp[0]&0xF)<<4) & 0xFF)|((tor_tmp[0]>>8) & 0xFF));
        send2[0].Data[7] = (BYTE)(tor_tmp[0] & 0xFF);

    mtx.unlock();
    
}
void PiceCan::msgTransmit1(void) 
{
    // std::cout<<"[0]:"<<send->Data[0]<<"[1]:"<<send->Data[1]<<"[2]:"<<send->Data[2]<<"[3]:"<<send->Data[3]<<
    // "[4]:"<<send->Data[4]<<"[5]:"<<send->Data[5]<<"[6]:"<<send->Data[6]<<"[7]:"<<send->Data[7]<<std::endl;
    VCI_Transmit(VCI_USBCAN2, board, dev_id, send,1);
}

void PiceCan::msgTransmit2(void) 
{
    VCI_Transmit(VCI_USBCAN2, board, dev_id, send1,1);
}

void PiceCan::msgTransmit3(void) 
{
    VCI_Transmit(VCI_USBCAN2, board, dev_id, send2,1);
}
std::mutex mtx3;
double * PiceCan::dataRece(void)
{
    if((reclen=VCI_Receive(VCI_USBCAN2,board,dev_id,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
    {
            p_int1=( rec[0].Data[1] << 8) |( rec[0].Data[2]);
            v_int1=(rec[0].Data[3]<<4)|((rec[0].Data[4] & 0xf0)>>4);             
            t_int1=((rec[0].Data[4]&0x0F)<<8)|(rec[0].Data[5]);

            id1=(rec[0].Data[0]<<4);

            pos1 = uint_to_float(p_int1, -12.5, 12.5, 16);
            vel1= uint_to_float(v_int1, -30.0f,30.0f, 12);
            toq1=  uint_to_float(t_int1, -10.0f, 10.0f, 12);	
            rec_seq=rec[0].TimeStamp;
            
            // printf("\n");
            // printf("board:%d _board_equNum:%d dev:%d  id: 0x%04x  angle:%f velocity:%f torque:%f  TimeStamp:%d\n",
                    // board,_board_equNum,dev_id, id1,pos1 * 180 / 3.14,vel1,toq1,rec_seq);	
                mtx3.lock();
                if(_board_equNum==56)   
                {
                    if(dev_id==1) //rb
                    {
                        if(id1==16)
                        {
                            ret_pos[0]=pos1;  
                            ret_vel[0]=vel1;
                            ret_tor[0]=toq1;
                            ret_seq[0]=rec_seq;
                            accum_data_order[0]++;
                            // std::cout<<"rb_m1-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;
                        }
                        else if(id1==32)
                        {
                            ret_pos[1]=pos1 ;  
                            ret_vel[1]=vel1;
                            ret_tor[1]=toq1;
                            ret_seq[1]=rec_seq; 
                            accum_data_order[1]++;
                            // std::cout<<"rb_m2-> p:"<<ret_pos[1]<<" v:"<<ret_vel[1]<<" t:"<<ret_tor[1]<<" seq:"<<ret_seq[1]<<std::endl;                      
                        }
                        else if(id1==48)
                        {
                            ret_pos[2]=pos1 *knee_offset ;  
                            ret_vel[2]=vel1;
                            ret_tor[2]=toq1;
                            ret_seq[2]=rec_seq;
                            accum_data_order[2]++;
                            // std::cout<<"rb_m3-> p:"<<ret_pos[2]<<" v:"<<ret_vel[2]<<" t:"<<ret_tor[2]<<" seq:"<<ret_seq[2]<<std::endl;                             
                        }
                    }
                    else if(dev_id==0) //lb
                    {
                        if(id1==16)
                        {
                            ret_pos[0]=pos1;  
                            ret_vel[0]=vel1;
                            ret_tor[0]=toq1;
                            ret_seq[0]=rec_seq;
                            accum_data_order[0]++;
                            // std::cout<<"lb_m1-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;
                        }
                        else if(id1==32)
                        {
                            ret_pos[1]=pos1 ;  
                            ret_vel[1]=vel1;
                            ret_tor[1]=toq1;
                            ret_seq[1]=rec_seq;
                            accum_data_order[1]++;
                            // std::cout<<"lb_m2-> p:"<<ret_pos[1]<<" v:"<<ret_vel[1]<<" t:"<<ret_tor[1]<<" seq:"<<ret_seq[1]<<std::endl;                      
                        }
                        else if(id1==48)//288
                        {
                            ret_pos[2]=pos1 *knee_offset;  
                            ret_vel[2]=vel1;
                            ret_tor[2]=toq1;
                            ret_seq[2]=rec_seq; 
                            accum_data_order[2]++;
                            // std::cout<<"lb_m3-> p:"<<ret_pos[2]<<" v:"<<ret_vel[2]<<" t:"<<ret_tor[2]<<" seq:"<<ret_seq[2]<<std::endl;                             
                        }
                    }
                    // printf("56\n");
                }

                if(_board_equNum==49)
                {
                    if(dev_id==1)   //rf
                    {
                        if(id1==16)
                        {
                            ret_pos[0]=pos1;  
                            ret_vel[0]=vel1;
                            ret_tor[0]=toq1;
                            accum_data_order[0]++;
                            ret_seq[0]=rec_seq;
                            // std::cout<<"rf_m1-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;

                        }
                        else if(id1==32)
                        {
                            ret_pos[1]=pos1 ;  
                            ret_vel[1]=vel1;
                            ret_tor[1]=toq1;
                            accum_data_order[1]++;
                            ret_seq[1]=rec_seq;    
                            // std::cout<<"rf_m2-> p:"<<ret_pos[1]<<" v:"<<ret_vel[1]<<" t:"<<ret_tor[1]<<" seq:"<<ret_seq[1]<<std::endl;                      

                        }
                        else if(id1==48)
                        {
                            ret_pos[2]=pos1 *knee_offset;  
                            ret_vel[2]=vel1;
                            ret_tor[2]=toq1;
                            accum_data_order[2]++;
                            ret_seq[2]=rec_seq;   
                            // std::cout<<"rf_m3-> p:"<<ret_pos[2]<<" v:"<<ret_vel[2]<<" t:"<<ret_tor[2]<<" seq:"<<ret_seq[2]<<std::endl;                             

                        }
                    }
                    else if(dev_id==0)  //lf
                    {
                        if(id1==16)//128 16
                        {
                            ret_pos[0]=pos1;  
                            ret_vel[0]=vel1;
                            ret_tor[0]=toq1;
                            accum_data_order[0]++;
                            ret_seq[0]=rec_seq;
                            // std::cout<<"lf_m1-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;

                        }
                        else if(id1==32)//144 32
                        {
                            ret_pos[1]=pos1;  
                            ret_vel[1]=vel1;
                            ret_tor[1]=toq1;
                            accum_data_order[1]++;
                            ret_seq[1]=rec_seq;      
                            // std::cout<<"lf_m2-> p:"<<ret_pos[1]<<" v:"<<ret_vel[1]<<" t:"<<ret_tor[1]<<" seq:"<<ret_seq[1]<<std::endl;                      

                        }
                        else if(id1==48)//160 48 
                        {
                            ret_pos[2]=pos1*knee_offset;  
                            ret_vel[2]=vel1;
                            ret_tor[2]=toq1;
                            accum_data_order[2]++;
                            ret_seq[2]=rec_seq;     
                            // std::cout<<"lf_m3-> p:"<<ret_pos[2]<<" v:"<<ret_vel[2]<<" t:"<<ret_tor[2]<<" seq:"<<ret_seq[2]<<std::endl;                             

                        }
                    }
                    // printf("49\n");
                }

                if(_board_equNum==50)
                {
                    if(dev_id==1)  //rm
                    {
                        if(id1==16) 
                        {
                            ret_pos[0]=pos1 ;  
                            ret_vel[0]=vel1;
                            ret_tor[0]=toq1;
                            accum_data_order[0]++;
                            ret_seq[0]=rec_seq;
                            // std::cout<<"rm_m1-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;
                        }
                        else if(id1==32)
                        {
                            ret_pos[1]=pos1;   
                            ret_vel[1]=vel1;
                            ret_tor[1]=toq1;
                            accum_data_order[1]++;
                            ret_seq[1]=rec_seq;
                            // std::cout<<"rm_m2-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;
                        }
                        else if(id1==48)
                        {
                            ret_pos[2]=pos1*knee_offset;  
                            ret_vel[2]=vel1;
                            ret_tor[2]=toq1;
                            accum_data_order[2]++;
                            ret_seq[2]=rec_seq;
                            // std::cout<<"rm_m3-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;
                        }
                    }
                    else if(dev_id==0)  //lm
                    {
                        if(id1==16)
                        {
                            ret_pos[0]=pos1;  
                            ret_vel[0]=vel1;
                            ret_tor[0]=toq1;
                            accum_data_order[0]++;
                            ret_seq[0]=rec_seq;
                            // std::cout<<"lm_m1-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;
                        }
                        else if(id1==32)
                        {
                            ret_pos[1]=pos1;  
                            ret_vel[1]=vel1;
                            ret_tor[1]=toq1;
                            accum_data_order[1]++;
                            ret_seq[1]=rec_seq;
                            // std::cout<<"lm_m2-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;
                        }
                        else if(id1==48)
                        {
                            ret_pos[2]=pos1*knee_offset;  
                            ret_vel[2]=vel1;
                            ret_tor[2]=toq1;
                            accum_data_order[2]++;
                            ret_seq[2]=rec_seq;
                            // std::cout<<"lm_m3-> p:"<<ret_pos[0]<<" v:"<<ret_vel[0]<<" t:"<<ret_tor[0]<<" seq:"<<ret_seq[0]<<std::endl;
                        }
                    }
                    // printf("50\n");
                }
                mtx3.unlock();
    }

    return RetData;
}







