#ifndef _PICECAN_H
#define _PICECAN_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <mutex>
#include<iostream>
#include <fstream>
#include<iomanip>
#include<stdlib.h>
////文件版本：v2.02 20190609
//接口卡类型定义

#define VCI_USBCAN1		3
#define VCI_USBCAN2		4
#define VCI_USBCAN2A		4

#define VCI_USBCAN_E_U 		20
#define VCI_USBCAN_2E_U 	21

//函数调用返回状态值
#define	STATUS_OK					1
#define STATUS_ERR					0
	
#define USHORT unsigned short int
#define BYTE unsigned char
#define CHAR char
#define UCHAR unsigned char
#define UINT unsigned int
#define DWORD unsigned int
#define PVOID void*
#define ULONG unsigned int
#define INT int
#define UINT32 UINT
#define LPVOID void*
#define BOOL BYTE
#define TRUE 1
#define FALSE 0

//1.ZLGCAN系列接口卡信息的数据类型。
typedef  struct  _VCI_BOARD_INFO{
		USHORT	hw_Version;
		USHORT	fw_Version;
		USHORT	dr_Version;
		USHORT	in_Version;
		USHORT	irq_Num;
		BYTE	can_Num;
		CHAR	str_Serial_Num[20];
		CHAR	str_hw_Type[40];
		USHORT	Reserved[4];
} VCI_BOARD_INFO,*PVCI_BOARD_INFO; 

//2.定义CAN信息帧的数据类型。
typedef  struct  _VCI_CAN_OBJ{
	UINT	ID=0;
	UINT	TimeStamp=0;
	BYTE	TimeFlag=0;
	BYTE	SendType=0;
	BYTE	RemoteFlag=0;//是否是远程帧
	BYTE	ExternFlag=0;//是否是扩展帧
	BYTE	DataLen=0;
	BYTE	Data[8]={0};
	BYTE	Reserved[3]={0};
}VCI_CAN_OBJ,*PVCI_CAN_OBJ;

//3.定义初始化CAN的数据类型
typedef struct _INIT_CONFIG{
	DWORD	AccCode;
	DWORD	AccMask;
	DWORD	Reserved;
	UCHAR	Filter;
	UCHAR	Timing0;	
	UCHAR	Timing1;	
	UCHAR	Mode;
}VCI_INIT_CONFIG,*PVCI_INIT_CONFIG;

///////// new add struct for filter /////////
typedef struct _VCI_FILTER_RECORD{
	DWORD ExtFrame;	//是否为扩展帧
	DWORD Start;
	DWORD End;
}VCI_FILTER_RECORD,*PVCI_FILTER_RECORD;

#ifdef __cplusplus
#define EXTERN_C  extern "C"
#else
#define EXTERN_C
#endif

EXTERN_C DWORD VCI_OpenDevice(DWORD DeviceType,DWORD DeviceInd,DWORD Reserved);
EXTERN_C DWORD VCI_CloseDevice(DWORD DeviceType,DWORD DeviceInd);
EXTERN_C DWORD VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);

EXTERN_C DWORD VCI_ReadBoardInfo(DWORD DeviceType,DWORD DeviceInd,PVCI_BOARD_INFO pInfo);

EXTERN_C DWORD VCI_SetReference(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,DWORD RefType,PVOID pData);

EXTERN_C ULONG VCI_GetReceiveNum(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
EXTERN_C DWORD VCI_ClearBuffer(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);

EXTERN_C DWORD VCI_StartCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
EXTERN_C DWORD VCI_ResetCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);

EXTERN_C ULONG VCI_Transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,UINT Len);
EXTERN_C ULONG VCI_Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pReceive,UINT Len,INT WaitTime);

EXTERN_C DWORD  VCI_UsbDeviceReset(DWORD DevType,DWORD DevIndex,DWORD Reserved);
EXTERN_C DWORD  VCI_FindUsbDevice2(PVCI_BOARD_INFO pInfo);

class PiceCan
{
    private:
		double pos1,vel1,toq1;
		int id1, p_int1,v_int1,t_int1;
		int rec_seq=0;
		int reclen=0;
        VCI_CAN_OBJ send[1],send1[1],send2[1];
        VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。set3000 *8data
        __uint16_t pos_tmp[3],vel_tmp[3],kp_tmp[3],kd_tmp[3],tor_tmp[3];
        double RetData[4]={0};
		int dev_id;
		int board;

		int _ret_board[3],_ret_dev[3],_ret_id[3];

		// double ret_pos[3],ret_vel[3],ret_tor[3],ret_seq[3],accum_data_order[3]={0};

		int _board_equNum=0;
    public:
		std::mutex mtx;
        double knee_offset=0.6429;
		int mtx_test_data=0;
        PiceCan(/* args */);
        ~PiceCan();
		double ret_pos[3],ret_vel[3],ret_tor[3],ret_seq[3],accum_data_order[3]={0};
        void createCanDev(int board,int dev_id,int motorId1,int motorId2,int motorId3);
        double uint_to_float(int x_int, double x_min, double x_max, int bits);
        int float_to_uint(double x, double x_min, double x_max, int bits);
        void loadSenMsgEntCloLoo(void);
        void loadSenMsgExiCloLoo(void);
        void loadSenMsgSetZeroPoint(void);
        void loaSenMsgData(double  * pos,double   * vel,double   * KP,double  * KD,double   * trop);
		void loaSenMsgData1(double  pos,double   vel,double   KP,double  KD,double trop);
		void loaSenMsgData2(double  pos,double   vel,double   KP,double  KD,double trop);
		void loaSenMsgData3(double  pos,double   vel,double   KP,double  KD,double trop);
        void msgTransmit1(void);
		void msgTransmit2(void);
		void msgTransmit3(void);
        double * dataRece(void);
		void getBoardEquiNumber(int board_equNum)
		{
			_board_equNum=board_equNum;
		}
		// double * retPos(void)
		// {
		// 	return ret_pos;
		// }
		// double * retVel(void)
		// {
		// 	return ret_vel;
		// }
		// double * retTor(void)
		// {
		// 	return ret_tor;
		// }
		// double * retSeq(void)
		// {
		// 	return ret_seq;
		// }

};






#endif