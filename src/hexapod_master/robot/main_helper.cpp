/*!
 * @file main_helper.cpp
 * @brief main_helper在main函数中调用，用来跑我们的机器人。
 * @brief 在robot_main.h中定义了宏，跟据宏定义控制机器人的功能、模式，请细看。很重要！！！请前往查看！
 * @author lcc
 * @date update at : 20230329
 */
#include "./robot_master/robot_main.h"
#include "main_helper.h"
#include "../common/utilities/timer.h"
#include"./hardware/hardware_bridge.h"

#if DEBUG==1
	KeyBoardControl KeyBoardCtrl_test;
	//lcc 20230329:键盘控制的线程
	void *remote_monitoraaa(void* arg)
	{	
			printf("remote_monitor thread is :%lu,pid:%d\n",pthread_self(),getpid());
			while(1)
			{	
					KeyBoardCtrl_test.scanKeyValue();
			}
	}
#endif

int main_helper(int argc, char *argv[])
{
	#if HARD_WARE==1      //lcc 20230329：  hard_ware定义在 ./robot_master/robot_main.h 中，用来决定是否开启机器人硬件
		int *board_num=HarBri.boardOrder();
		int *board_equNum=HarBri.boardEquNum();
		printf("can1:%d   can2:%d   can3:%d\n" ,board_num[0],board_num[1],board_num[2]);
		HarBri.creatCanBoard0(board_num[0],board_equNum[0]); //rf lf  49
		HarBri.creatCanBoard1(board_num[1],board_equNum[1]);//lm rm 50
		HarBri.creatCanBoard2(board_num[2],board_equNum[2]);//rb lb 56
	#endif

	#if DEBUG==0
		Hexapod rob;      //lcc 20230329: 定义一个名叫rob的六足机器人实例
		rob.run(argc,argv); //lcc 20230329: rob开始运行
	#endif 

	#if DEBUG==1
		pthread_t RemoteControlThraaa;
		pthread_create(&RemoteControlThraaa,NULL,remote_monitoraaa,NULL);
		while (1)
		{
			switch (KeyBoardCtrl_test.retKeyValue())
			{
					case 't':     
							{
								for(int i=0;i<3;i++)
								{
									HarBri.lf_des_pos(i)=30*HarBri._RAD1; HarBri.lm_des_pos(i)=30*HarBri._RAD1; 
									HarBri.lb_des_pos(i)=30*HarBri._RAD1;
									HarBri.rf_des_pos(i)=30*HarBri._RAD1; HarBri.rm_des_pos(i)=30*HarBri._RAD1;
									HarBri.rb_des_pos(i)=30*HarBri._RAD1;

									HarBri.lf_des_tor(i)=0; HarBri.lm_des_tor(i)=0; HarBri.lb_des_tor(i)=0;
									HarBri.rf_des_tor(i)=0; HarBri.rm_des_tor(i)=0; HarBri.rb_des_tor(i)=0;

									HarBri.lf_des_vel(i)=0; HarBri.lm_des_vel(i)=0; HarBri.lb_des_vel(i)=0;
									HarBri.rf_des_vel(i)=0; HarBri.rm_des_vel(i)=0; HarBri.rb_des_vel(i)=0;

									HarBri.lf_des_kp(i)=20; HarBri.lm_des_kp(i)=20; HarBri.lb_des_kp(i)=20;
									HarBri.rf_des_kp(i)=20; HarBri.rm_des_kp(i)=20; HarBri.rb_des_kp(i)=20;

									HarBri.lf_des_kd(i)=1; HarBri.lm_des_kd(i)=1; HarBri.lb_des_kd(i)=1;
									HarBri.rf_des_kd(i)=1; HarBri.rm_des_kd(i)=1; HarBri.rb_des_kd(i)=1;
								}
								HarBri.lfDataLoading(HarBri.lf_des_pos, HarBri.lf_des_vel, HarBri.lf_des_tor, HarBri.lf_des_kp, HarBri.lf_des_kd);
								HarBri.lmDataLoading(HarBri.lm_des_pos, HarBri.lm_des_vel, HarBri.lm_des_tor, HarBri.lm_des_kp, HarBri.lm_des_kd);
								HarBri.lbDataLoading(HarBri.lb_des_pos, HarBri.lb_des_vel, HarBri.lb_des_tor, HarBri.lb_des_kp, HarBri.lb_des_kd);
								HarBri.rfDataLoading(HarBri.rf_des_pos, HarBri.rf_des_vel, HarBri.rf_des_tor, HarBri.rf_des_kp, HarBri.rf_des_kd);
								HarBri.rmDataLoading(HarBri.rm_des_pos, HarBri.rm_des_vel, HarBri.rm_des_tor, HarBri.rm_des_kp, HarBri.rm_des_kd);
								HarBri.rbDataLoading(HarBri.rb_des_pos, HarBri.rb_des_vel, HarBri.rb_des_tor, HarBri.rb_des_kp, HarBri.rb_des_kd);
							}
							break; 
					case 'i':     //lcc 20230330:进入闭环
							{
								HarBri.robDataLoadEnterCloseLoop();   
							}
							break; 
					case 'p':     //lcc 20230330:设置零点
							{
								HarBri.robDataLoadSetZeroPoint();
							}
							break; 
					case 'o':    //lcc 20230330:退出闭环
							{
								HarBri.robDataLoadExitCloesLoop();
							}
							break;
					default: ;
							break;
			} 
			HarBri.dataRecAllAndSeqUpdate();
			HarBri.dataIntegrat();
			HarBri.showAllData();
			usleep(10000);
		}
	#endif

    return 0;
}


