#include"hardware_bridge.h"
HardwareBridge HarBri;
int HardwareBridge::global_send_thread_flag=0;
VCI_BOARD_INFO pInfo1 [50];
int delay_time=1;

void *receive_func0(void* param)  //接收线程。
{  
    printf("receive_func0 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	while(1)
	{	
        // HarBri.CanDevice0.mtx_test_data++;
        // std::cout<<"receive_func0  mtx_test_data:  "<<HarBri.CanDevice0.mtx_test_data<<std::endl;
        // HarBri.CanDevice0.mtx.lock();
		HarBri.CanDevice0.dataRece();
        // HarBri.CanDevice0.mtx.unlock();
	}
}

void *receive_func1(void* param)  //接收线程。
{   printf("receive_func1 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	while(1)
	{	
        // HarBri.CanDevice1.mtx.lock();
		HarBri.CanDevice1.dataRece();
        // HarBri.CanDevice1.mtx.unlock(); 
	}
}

void *receive_func2(void* param)  //接收线程。
{   printf("receive_func2 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	while(1)
	{	
        HarBri.CanDevice2.dataRece(); 
	}
}

void *receive_func3(void* param)  //接收线程。
{   printf("receive_func3 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	while(1)
	{	
        HarBri.CanDevice3.dataRece(); 
	}
}

void *receive_func4(void* param)  //接收线程。
{   printf("receive_func4 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	while(1)
	{	
        HarBri.CanDevice4.dataRece(); 
	}
}

void *receive_func5(void* param)  //接收线程。
{   printf("receive_func5 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	while(1)
	{	
        HarBri.CanDevice5.dataRece(); 
	}
}
    //    CanDevice0.createCanDev(testnum0,1, 1,2,3);//rf
    // CanDevice1.createCanDev(testnum0,0, 1,2,3);//lf 
void *Send0(void* param)  //线程。
{   printf("Send0 thread is :%lu,pid:%d\n",pthread_self(),getpid());

    int a=0, xx_can_dev_seq_last[3]={0},printf_flag=0;
    // int iii=0;
    // while (HardwareBridge::global_send_thread_flag==0)  
    // {
    //     iii++;
    //     if(iii>100000) iii=0;
    //     // printf("Send0  thread is wating.....! \n");
    // }
	// while(HardwareBridge::global_send_thread_flag)  //lcc 20230425: 发送前，检查角度是否突变
	while(1)  //lcc 20230425: 发送前，检查角度是否突变
	{
        // printf("Send0  thread enter! \n");
        // printf("Send0 HardwareBridge::global_send_thread_flag:%d\n",HardwareBridge::global_send_thread_flag);
        if(  
            HarBri.rf_can_dev_seq[0]!=xx_can_dev_seq_last[0] &&  
            HarBri.rf_can_dev_seq[1]!=xx_can_dev_seq_last[1] &&
            HarBri.rf_can_dev_seq[2]!=xx_can_dev_seq_last[2] &&
            HarBri.rf_act_pos(0)!=0 && 
            HarBri.rf_act_pos(1)!=0 && 
            HarBri.rf_act_pos(2)!=0 && 
            a<=120 )
        {
            xx_can_dev_seq_last[0]=HarBri.rf_can_dev_seq[0];
            xx_can_dev_seq_last[1]=HarBri.rf_can_dev_seq[1];
            xx_can_dev_seq_last[2]=HarBri.rf_can_dev_seq[2];
            a++;
            // printf("rfa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=120 && a<=130)
        {
            HarBri.RfProtect.sendDataConPro(3,HarBri.rf_act_pos,10*HarBri._RAD1);
            for(int i=0;i<3;i++)
            {
                HarBri.rf_des_pos(i)=HarBri.rf_act_pos(i);
            }
            a++;
            // printf("rfaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=130)
        {   

            //test program
            // HarBri.rf_des_pos[1]=HarBri.rf_des_pos[1]+6*HarBri._RAD1;

            for(int i=0;i<3;i++)
            {
                if( isnan(HarBri.rf_des_pos(i)) or 
                    isnanf(HarBri.rf_des_pos(i)) or
                    isinf(HarBri.rf_des_pos(i)) or 
                    isinff(HarBri.rf_des_pos(i)) 
                   ) 
                    HarBri.RfProtect.StopFlag=false;
            }
            if(printf_flag==0)
            {
                HarBri.rf_protect_init_flag=1; 
                printf("rf_protect_init_flag=1\n");
                printf_flag=1;

            }
            HarBri.RfProtect.sendDataConPro(3,HarBri.rf_des_pos,10*HarBri._RAD1);
            // printf("rfaaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        
        if(HarBri.RfProtect.StopFlag==true)
        {
            HarBri.CanDevice0.msgTransmit1();
            HarBri.CanDevice0.msgTransmit2();
            HarBri.CanDevice0.msgTransmit3();
            // printf("ssasasasasfasf\n");
        }
        else
        {
            HarBri.CanDevice0.loadSenMsgExiCloLoo();
            HarBri.CanDevice0.msgTransmit1();
            HarBri.CanDevice0.msgTransmit2();
            HarBri.CanDevice0.msgTransmit3();
            printf("\n   des_pos erro;  报错在 *Send0(void* param)线程 \n");
        }
	}
}
void *Send1(void* param)  //线程。
{   printf("Send1 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	
    int a=0, xx_can_dev_seq_last[3]={0},printf_flag=0;
    // while (HardwareBridge::global_send_thread_flag==0)  
    // {
    //     // printf("Send1  thread is wating....... \n");
    // }
	// while(HardwareBridge::global_send_thread_flag)  //lcc 20230425: 发送前，检查角度是否突变
	while(1)  //lcc 20230425: 发送前，检查角度是否突变
	{	
        // printf("Send1  thread enter! \n");
        if(  
            HarBri.lf_can_dev_seq[2]!=xx_can_dev_seq_last[0] &&  
            HarBri.lf_can_dev_seq[1]!=xx_can_dev_seq_last[1] &&
            HarBri.lf_can_dev_seq[2]!=xx_can_dev_seq_last[2] &&
            HarBri.lf_act_pos(0)!=0 && 
            HarBri.lf_act_pos(1)!=0 && 
            HarBri.lf_act_pos(2)!=0 && 
            a<=120 )
        {
            xx_can_dev_seq_last[0]=HarBri.lf_can_dev_seq[0];
            xx_can_dev_seq_last[1]=HarBri.lf_can_dev_seq[1];
            xx_can_dev_seq_last[2]=HarBri.lf_can_dev_seq[2];
            a++;
            // printf("lfa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=120 && a<=130)
        {
            HarBri.LfProtect.sendDataConPro(0,HarBri.lf_act_pos,10*HarBri._RAD1);
            for(int i=0;i<3;i++)
            {
                HarBri.lf_des_pos(i)=HarBri.lf_act_pos(i);
            }
            a++;
            // printf("lfaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=130)
        {   
            if(printf_flag==0)
            {
                HarBri.lf_protect_init_flag=1; 
                printf("lf_protect_init_flag=1\n");
                printf_flag=1;

            }

            //test program
            // HarBri.lf_des_pos[1]=HarBri.lf_des_pos[1]+6*HarBri._RAD1;

            for(int i=0;i<3;i++)
            {
                if( isnan(HarBri.lf_des_pos(i)) or 
                    isnanf(HarBri.lf_des_pos(i)) or
                    isinf(HarBri.lf_des_pos(i)) or 
                    isinff(HarBri.lf_des_pos(i)) 
                   ) 
                    HarBri.LfProtect.StopFlag=false;
            }

            HarBri.LfProtect.sendDataConPro(0,HarBri.lf_des_pos,10*HarBri._RAD1);
            // printf("rfaaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        
        if(HarBri.LfProtect.StopFlag==true)
        {
            HarBri.CanDevice1.msgTransmit1();
            HarBri.CanDevice1.msgTransmit2();
            HarBri.CanDevice1.msgTransmit3();
        }
        else
        {
            HarBri.CanDevice1.loadSenMsgExiCloLoo();
            HarBri.CanDevice1.msgTransmit1();
            HarBri.CanDevice1.msgTransmit2();
            HarBri.CanDevice1.msgTransmit3();
            printf("\n   des_pos erro;  报错在 *Send1(void* param)线程 \n");
        }
	}
    // while(HardwareBridge::global_send_thread_flag)
	// {	
    //     HarBri.LfProtect.sendDataConPro(&HarBri.lf,HarBri.lf_des_pos,5);
    //     HarBri.CanDevice1.msgTransmit1();
    //     HarBri.CanDevice1.msgTransmit2();
    //     HarBri.CanDevice1.msgTransmit3();
    //     // usleep(delay_time);
	// }
}
    // CanDevice2.createCanDev(testnum1,0, 1,2,3);//lm
    // CanDevice3.createCanDev(testnum1,1, 1,2,3);//rm
void *Send2(void* param)  //线程。
{   printf("Send2 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	
    int a=0, xx_can_dev_seq_last[3]={0},printf_flag=0;
    // while (HardwareBridge::global_send_thread_flag==0)  
    // {
    //     // printf("Send2  thread is wating....... \n");
    // }
	// while(HardwareBridge::global_send_thread_flag)  //lcc 20230425: 发送前，检查角度是否突变
	while(1)  //lcc 20230425: 发送前，检查角度是否突变
	{	
        // printf("Send2  thread enter! \n");
        if(  
            HarBri.lm_can_dev_seq[2]!=xx_can_dev_seq_last[0] &&  
            HarBri.lm_can_dev_seq[1]!=xx_can_dev_seq_last[1] &&
            HarBri.lm_can_dev_seq[2]!=xx_can_dev_seq_last[2] &&
            HarBri.lm_act_pos(0)!=0 && 
            HarBri.lm_act_pos(1)!=0 && 
            HarBri.lm_act_pos(2)!=0 && 
            a<=120 )
        {
            xx_can_dev_seq_last[0]=HarBri.lm_can_dev_seq[0];
            xx_can_dev_seq_last[1]=HarBri.lm_can_dev_seq[1];
            xx_can_dev_seq_last[2]=HarBri.lm_can_dev_seq[2];
            a++;
            // printf("lfa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=120 && a<=130)
        {
            HarBri.LmProtect.sendDataConPro(1,HarBri.lm_act_pos,10*HarBri._RAD1);
            for(int i=0;i<3;i++)
            {
                HarBri.lm_des_pos(i)=HarBri.lm_act_pos(i);
            }
            a++;
            // printf("lfaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=130)
        {   

            if(printf_flag==0)
            {
                HarBri.lm_protect_init_flag=1; 
                printf("lm_protect_init_flag=1\n");
                printf_flag=1;

            }

            //test program
            // HarBri.lm_des_pos[1]=HarBri.lm_des_pos[1]+6*HarBri._RAD1;

            for(int i=0;i<3;i++)
            {
                if( isnan(HarBri.lm_des_pos(i)) or 
                    isnanf(HarBri.lm_des_pos(i)) or
                    isinf(HarBri.lm_des_pos(i)) or 
                    isinff(HarBri.lm_des_pos(i)) 
                   ) 
                    HarBri.LmProtect.StopFlag=false;
            }
            HarBri.LmProtect.sendDataConPro(1,HarBri.lm_des_pos,10*HarBri._RAD1);
            // printf("rfaaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        
        if(HarBri.LmProtect.StopFlag==true)
        {
            HarBri.CanDevice2.msgTransmit1();
            HarBri.CanDevice2.msgTransmit2();
            HarBri.CanDevice2.msgTransmit3();
        }
        else
        {
            HarBri.CanDevice2.loadSenMsgExiCloLoo();
            HarBri.CanDevice2.msgTransmit1();
            HarBri.CanDevice2.msgTransmit2();
            HarBri.CanDevice2.msgTransmit3();
            printf("\n   des_pos erro;  报错在 *Send2(void* param)线程 \n");
        }
	}
    // while(HardwareBridge::global_send_thread_flag)
	// {	
    //     HarBri.LmProtect.sendDataConPro(&HarBri.lm,HarBri.lm_des_pos,5);
    //     HarBri.CanDevice2.msgTransmit1();
    //     HarBri.CanDevice2.msgTransmit2();
    //     HarBri.CanDevice2.msgTransmit3();
    //     // usleep(delay_time);
	// }
}

void *Send3(void* param)  //线程。
{   printf("Send3 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	
    int a=0, xx_can_dev_seq_last[3]={0},printf_flag=0;
    // while (HardwareBridge::global_send_thread_flag==0)  
    // {
    //     // printf("Send3  thread is wating....... \n");
    // }
	// while(HardwareBridge::global_send_thread_flag)  //lcc 20230425: 发送前，检查角度是否突变
	while(1)  //lcc 20230425: 发送前，检查角度是否突变
	{	
        // printf("Send3  thread enter! \n");
        if(  
            HarBri.rm_can_dev_seq[2]!=xx_can_dev_seq_last[0] &&  
            HarBri.rm_can_dev_seq[1]!=xx_can_dev_seq_last[1] &&
            HarBri.rm_can_dev_seq[2]!=xx_can_dev_seq_last[2] &&
            HarBri.rm_act_pos(0)!=0 && 
            HarBri.rm_act_pos(1)!=0 && 
            HarBri.rm_act_pos(2)!=0 && 
            a<=120 )
        {
            xx_can_dev_seq_last[0]=HarBri.rm_can_dev_seq[0];
            xx_can_dev_seq_last[1]=HarBri.rm_can_dev_seq[1];
            xx_can_dev_seq_last[2]=HarBri.rm_can_dev_seq[2];
            a++;
            // printf("lfa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=120 && a<=130)
        {
            HarBri.RmProtect.sendDataConPro(4,HarBri.rm_act_pos,10*HarBri._RAD1);
            for(int i=0;i<3;i++)
            {
                HarBri.rm_des_pos(i)=HarBri.rm_act_pos(i);
            }
            a++;
            // printf("lfaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=130)
        {   

            if(printf_flag==0)
            {
                HarBri.rm_protect_init_flag=1; 
                printf("rm_protect_init_flag=1\n");
                printf_flag=1;

            }

            //test program
            // HarBri.rm_des_pos[1]=HarBri.rm_des_pos[1]+6*HarBri._RAD1;

            for(int i=0;i<3;i++)
            {
                if( isnan(HarBri.rm_des_pos(i)) or 
                    isnanf(HarBri.rm_des_pos(i)) or
                    isinf(HarBri.rm_des_pos(i)) or 
                    isinff(HarBri.rm_des_pos(i)) 
                   ) 
                    HarBri.RmProtect.StopFlag=false;
            }
            HarBri.RmProtect.sendDataConPro(4,HarBri.rm_des_pos,10*HarBri._RAD1);
            // printf("rfaaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        
        if(HarBri.RmProtect.StopFlag==true)
        {
            HarBri.CanDevice3.msgTransmit1();
            HarBri.CanDevice3.msgTransmit2();
            HarBri.CanDevice3.msgTransmit3();
        }
        else
        {
            HarBri.CanDevice3.loadSenMsgExiCloLoo();
            HarBri.CanDevice3.msgTransmit1();
            HarBri.CanDevice3.msgTransmit2();
            HarBri.CanDevice3.msgTransmit3();
            printf("\n   des_pos erro;  报错在 *Send3(void* param)线程 \n");
        }
	}    
    // while(HardwareBridge::global_send_thread_flag)
	// {	
    //     HarBri.RmProtect.sendDataConPro(&HarBri.rm,HarBri.rm_des_pos,5);
    //     HarBri.CanDevice3.msgTransmit1();
    //     HarBri.CanDevice3.msgTransmit2();
    //     HarBri.CanDevice3.msgTransmit3();
    //     // usleep(delay_time);
	// }
}
    // CanDevice4.createCanDev(testnum2,1, 1,2,3);//rb
    // CanDevice5.createCanDev(testnum2,0, 1,2,3);//lb
void *Send4(void* param)  //线程。
{   printf("Send4 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	
    int a=0, xx_can_dev_seq_last[3]={0},printf_flag=0;
    // while (HardwareBridge::global_send_thread_flag==0)  
    // {
    //     // printf("Send4  thread is wating....... \n");
    // }
	// while(HardwareBridge::global_send_thread_flag)  //lcc 20230425: 发送前，检查角度是否突变
	while(1)  //lcc 20230425: 发送前，检查角度是否突变
	{	
        // printf("Send4  thread enter! \n");
        if(  
            HarBri.rb_can_dev_seq[2]!=xx_can_dev_seq_last[0] &&  
            HarBri.rb_can_dev_seq[1]!=xx_can_dev_seq_last[1] &&
            HarBri.rb_can_dev_seq[2]!=xx_can_dev_seq_last[2] &&
            HarBri.rb_act_pos(0)!=0 && 
            HarBri.rb_act_pos(1)!=0 && 
            HarBri.rb_act_pos(2)!=0 && 
            a<=120 )
        {
            xx_can_dev_seq_last[0]=HarBri.rb_can_dev_seq[0];
            xx_can_dev_seq_last[1]=HarBri.rb_can_dev_seq[1];
            xx_can_dev_seq_last[2]=HarBri.rb_can_dev_seq[2];
            a++;
            // printf("lfa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=120 && a<=130)
        {
            HarBri.RbProtect.sendDataConPro(5,HarBri.rb_act_pos,10*HarBri._RAD1);
            for(int i=0;i<3;i++)
            {
                HarBri.rb_des_pos(i)=HarBri.rb_act_pos(i);
            }
            a++;
            // printf("lfaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=130)
        {   
            if(printf_flag==0)
            {
                HarBri.rb_protect_init_flag=1; 
                printf("rb_protect_init_flag=1\n");
                printf_flag=1;

            }


            //test program
            // HarBri.rb_des_pos[1]=HarBri.rb_des_pos[1]+6*HarBri._RAD1;

            for(int i=0;i<3;i++)
            {
                if( isnan(HarBri.rb_des_pos(i)) or 
                    isnanf(HarBri.rb_des_pos(i)) or
                    isinf(HarBri.rb_des_pos(i)) or 
                    isinff(HarBri.rb_des_pos(i)) 
                   ) 
                    HarBri.RbProtect.StopFlag=false;
            }
            HarBri.RbProtect.sendDataConPro(5,HarBri.rb_des_pos,10*HarBri._RAD1);
            // printf("rfaaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        
        if(HarBri.RbProtect.StopFlag==true)
        {
            HarBri.CanDevice4.msgTransmit1();
            HarBri.CanDevice4.msgTransmit2();
            HarBri.CanDevice4.msgTransmit3();
        }
        else
        {
            HarBri.CanDevice4.loadSenMsgExiCloLoo();
            HarBri.CanDevice4.msgTransmit1();
            HarBri.CanDevice4.msgTransmit2();
            HarBri.CanDevice4.msgTransmit3();
            printf("\n   des_pos erro;  报错在 *Send4(void* param)线程 \n");
        }
	}
    // while(HardwareBridge::global_send_thread_flag)
	// {	
    //     HarBri.RbProtect.sendDataConPro(&HarBri.rb,HarBri.rb_des_pos,5);
    //     HarBri.CanDevice4.msgTransmit1();
    //     HarBri.CanDevice4.msgTransmit2();
    //     HarBri.CanDevice4.msgTransmit3(); 
    //     // usleep(delay_time);
	// }
}

void *Send5(void* param)  //线程。
{   printf("Send5 thread is :%lu,pid:%d\n",pthread_self(),getpid());
	
    int a=0, xx_can_dev_seq_last[3]={0},printf_flag=0;
    // while (HardwareBridge::global_send_thread_flag==0)  
    // {
    //     // printf("Send5  thread is wating....... \n");
    // }
	// while(HardwareBridge::global_send_thread_flag) //lcc 20230425: 发送前，检查角度是否突变
	while(1) //lcc 20230425: 发送前，检查角度是否突变
	{	
        // printf("Send5  thread enter! \n");
        if(  
            HarBri.lb_can_dev_seq[2]!=xx_can_dev_seq_last[0] &&  
            HarBri.lb_can_dev_seq[1]!=xx_can_dev_seq_last[1] &&
            HarBri.lb_can_dev_seq[2]!=xx_can_dev_seq_last[2] &&
            HarBri.lb_act_pos(0)!=0 && 
            HarBri.lb_act_pos(1)!=0 && 
            HarBri.lb_act_pos(2)!=0 && 
            a<=120 )
        {
            xx_can_dev_seq_last[0]=HarBri.lb_can_dev_seq[0];
            xx_can_dev_seq_last[1]=HarBri.lb_can_dev_seq[1];
            xx_can_dev_seq_last[2]=HarBri.lb_can_dev_seq[2];
            a++;
            // printf("lfa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=120 && a<=130)
        {
            HarBri.LbProtect.sendDataConPro(2,HarBri.lb_act_pos,10*HarBri._RAD1);
            for(int i=0;i<3;i++)
            {
                HarBri.lb_des_pos(i)=HarBri.lb_act_pos(i);
            }
            a++;
            // printf("lfaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        else if(a>=130)
        {   
            if(printf_flag==0)
            {
                HarBri.lb_protect_init_flag=1; 
                printf("lb_protect_init_flag=1\n");
                printf_flag=1;

                //test program
                // HarBri.lb_des_pos[1]=HarBri.lb_des_pos[1]+6*HarBri._RAD1;

            }

            for(int i=0;i<3;i++)
            {
                if( isnan(HarBri.lb_des_pos(i)) or 
                    isnanf(HarBri.lb_des_pos(i)) or
                    isinf(HarBri.lb_des_pos(i)) or 
                    isinff(HarBri.lb_des_pos(i)) 
                   ) 
                    HarBri.LbProtect.StopFlag=false;
            }

            HarBri.LbProtect.sendDataConPro(2,HarBri.lb_des_pos,10*HarBri._RAD1);
            // printf("rfaaa:%d xx_can_dev_seq_last:%d  %d  %d \n",a,xx_can_dev_seq_last[0],xx_can_dev_seq_last[1],xx_can_dev_seq_last[2]);
        }
        
        if(HarBri.LbProtect.StopFlag==true)
        {
            HarBri.CanDevice5.msgTransmit1();
            HarBri.CanDevice5.msgTransmit2();
            HarBri.CanDevice5.msgTransmit3();
        }
        else
        {   
            HarBri.CanDevice5.loadSenMsgExiCloLoo();
            HarBri.CanDevice5.msgTransmit1();
            HarBri.CanDevice5.msgTransmit2();
            HarBri.CanDevice5.msgTransmit3();
            printf("HarBri.lb_des_pos:%f %f %f  \n",HarBri.lb_des_pos[0],HarBri.lb_des_pos[1],HarBri.lb_des_pos[2]);
            printf("\n   des_pos erro;  报错在 *Send5(void* param)线程 \n");
        }
	}
    // while(HardwareBridge::global_send_thread_flag)
	// {	

    //     HarBri.LbProtect.sendDataConPro(&HarBri.lb,HarBri.lb_des_pos,5);
    //     HarBri.CanDevice5.msgTransmit1();
    //     HarBri.CanDevice5.msgTransmit2();
    //     HarBri.CanDevice5.msgTransmit3();
    //     // usleep(delay_time);
	// }
}

void HardwareBridge::openSendThread(int key)
{
    printf("key:%d\n",key);
    if(key==1)
    {
        pthread_t tSend0;
        pthread_create(&tSend0,NULL,Send0,NULL);    

        pthread_t tSend1;
        pthread_create(&tSend1,NULL,Send1,NULL);

        pthread_t tSend2;
        pthread_create(&tSend2,NULL,Send2,NULL);

        pthread_t tSend3;
        pthread_create(&tSend3,NULL,Send3,NULL);

        pthread_t tSend4;
        pthread_create(&tSend4,NULL,Send4,NULL);
        pthread_t tSend5;
        pthread_create(&tSend5,NULL,Send5,NULL);
    }

}

void HardwareBridge::creatCanBoard0(int testnum0,int equ_number)
{
    CanDevice0.getBoardEquiNumber(49);
    CanDevice1.getBoardEquiNumber(49);

    if(VCI_OpenDevice(VCI_USBCAN2,testnum0,0)==1)//打开设备
    {
        printf(">>open deivce success!\n");//打开设备成功
    }
    else
    {
        printf(">>open deivce error!\n");
        exit(1);
    }

    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;
    config.Filter=1;//接收所有帧
    config.Timing0=0x00;/*波特率1M  0x00  0x14*/
    config.Timing1=0x14;
    config.Mode=0;//正常模式

    if(VCI_InitCAN(VCI_USBCAN2,testnum0,0,&config)!=1)
    {
        printf(">>Init 0CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }
    if(VCI_StartCAN(VCI_USBCAN2,testnum0,0)!=1)
    {
        printf(">>Start 0CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }
    if(VCI_InitCAN(VCI_USBCAN2,testnum0,1,&config)!=1)
    {
        printf(">>Init 0can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);

    }
    if(VCI_StartCAN(VCI_USBCAN2,testnum0,1)!=1)
    {
        printf(">>Start 0can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);

    }

    CanDevice0.createCanDev(testnum0,1, 1,2,3);//rf
    CanDevice1.createCanDev(testnum0,0, 1,2,3);//lf 
    // CanDevice1.createCanDev(testnum0,0, 1,2,3);//lf          w

    pthread_t threadid0;
    pthread_create(&threadid0,NULL,receive_func0,NULL);
    // pthread_t tSend0;
    // pthread_create(&tSend0,NULL,Send0,NULL);

    pthread_t threadid1;
    pthread_create(&threadid1,NULL,receive_func1,NULL);
    // pthread_t tSend1;
    // pthread_create(&tSend1,NULL,Send1,NULL);

}

void HardwareBridge::creatCanBoard1(int testnum1,int equ_number)
{
    CanDevice2.getBoardEquiNumber(50);
    CanDevice3.getBoardEquiNumber(50);

    if(VCI_OpenDevice(VCI_USBCAN2,testnum1,0)==1)//打开设备
    {
        printf(">>open deivce1 success!\n");//打开设备成功
    }
    else
    {
        printf(">>open deivce1 error!\n");
        exit(1);
    }

    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;
    config.Filter=1;//接收所有帧
    config.Timing0=0x00;/*波特率1M  0x00  0x14*/
    config.Timing1=0x14;
    config.Mode=0;//正常模式

    if(VCI_InitCAN(VCI_USBCAN2,testnum1,0,&config)!=1)
    {
        printf(">>Init 1CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,1);
    }

    if(VCI_StartCAN(VCI_USBCAN2,testnum1,0)!=1)
    {
        printf(">>Start 1CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,1);

    }
    if(VCI_InitCAN(VCI_USBCAN2,testnum1,1,&config)!=1)
    {
        printf(">>Init 1can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,1);
    }
    if(VCI_StartCAN(VCI_USBCAN2,testnum1,1)!=1)
    {
        printf(">>Start 1can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,1);
    }      
    CanDevice2.createCanDev(testnum1,0, 1,2,3);//lm
    CanDevice3.createCanDev(testnum1,1, 1,2,3);//rm

    pthread_t threadid2;
    pthread_create(&threadid2,NULL,receive_func2,NULL);
    // pthread_t tSend2;
    // pthread_create(&tSend2,NULL,Send2,NULL);

    pthread_t threadid3;
    pthread_create(&threadid3,NULL,receive_func3,NULL);
    // pthread_t tSend3;
    // pthread_create(&tSend3,NULL,Send3,NULL);
}

void HardwareBridge::creatCanBoard2(int testnum2,int equ_number)
{
    CanDevice4.getBoardEquiNumber(56);
    CanDevice5.getBoardEquiNumber(56);

    if(VCI_OpenDevice(VCI_USBCAN2,testnum2,0)==1)//打开设备
    {
        printf(">>open deivce1 success!\n");//打开设备成功
    }
    else
    {
        printf(">>open deivce1 error!\n");
        exit(1);
    }

    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;
    config.Filter=1;//接收所有帧
    config.Timing0=0x00;/*波特率1M  0x00  0x14*/
    config.Timing1=0x14;
    config.Mode=0;//正常模式

    if(VCI_InitCAN(VCI_USBCAN2,testnum2,0,&config)!=1)
    {
        printf(">>Init 1CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,testnum2);
    }

    if(VCI_StartCAN(VCI_USBCAN2,testnum2,0)!=1)
    {
        printf(">>Start 1CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,testnum2);

    }
    if(VCI_InitCAN(VCI_USBCAN2,testnum2,1,&config)!=1)
    {
        printf(">>Init 1can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,testnum2);
    }
    if(VCI_StartCAN(VCI_USBCAN2,testnum2,1)!=1)
    {
        printf(">>Start 1can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,testnum2);
    }   
    CanDevice4.createCanDev(testnum2,1, 1,2,3);//rb
    CanDevice5.createCanDev(testnum2,0, 1,2,3);//lb
    pthread_t threadid4;
    pthread_create(&threadid4,NULL,receive_func4,NULL);
    // pthread_t tSend4;
    // pthread_create(&tSend4,NULL,Send4,NULL);

    pthread_t threadid5;
    pthread_create(&threadid5,NULL,receive_func5,NULL);
    // pthread_t tSend5;
    // pthread_create(&tSend5,NULL,Send5,NULL);
}

int * HardwareBridge::boardOrder(void)
{

        _board_num = VCI_FindUsbDevice2(pInfo1);

        printf(">>USBCAN DEVICE NUM: %d PCS \n", _board_num);

        for (int i = 0; i < 3; i++)
        {
                printf("pInfo1[%d].str_Serial_Num: %d\n", i, pInfo1[i].str_Serial_Num[10]);
                if(pInfo1[i].str_Serial_Num[10]==49)
                {  
                    _board_set_order[0]=i;
                    _ret_real_board_equnum[0]=49;
                    printf("_board_set_order[0]: %d\n",_board_set_order[0]);
                }

                if(pInfo1[i].str_Serial_Num[10]==50)
                {
                    _board_set_order[1]=i;
                    _ret_real_board_equnum[1]=50;
                    printf("_board_set_order[1]: %d\n",_board_set_order[1]);
                }

                if(pInfo1[i].str_Serial_Num[10]==56)
                {
                    _board_set_order[2]=i;
                    _ret_real_board_equnum[2]=56;
                    printf("_board_set_order[2]: %d\n",_board_set_order[2]);
                }
        }
        // printf("can1:12313   can2:  313131 can3:31231231\n");
        // printf("can1:%d   can2:%d   can3:%d \n" ,data[0],data[1],data[2]);

    return _board_set_order;

}

int * HardwareBridge::boardEquNum(void)
{
    return _ret_real_board_equnum;
}


