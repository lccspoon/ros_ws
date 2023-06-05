#include "neural_bezier_curve.h"

double * FourthPlane(double CPGXout,int CPGYout)
{   
  double v=2,h=10*2.5,s=10*4*2;
  static double positionOut[3];

  positionOut[0]=((3*s+4*v)/8.*CPGXout-((s+4*v)/8)*(pow(CPGXout,3)))*0.1;
  positionOut[1]=0;
  positionOut[2]=CPGYout*h*(1-2.*pow(CPGXout,2)+pow(CPGXout,4))*0.1;

  return positionOut;
}

double neural_bezier_curve::comb(double n,double m)  //%%c是一个数组,二项式的函数，杨辉三角系数
{
    int c=1;
    for(int jj=1; jj<=m; jj++) //jj=1:m
        c=c*(n-m+jj)/jj;
    // printf("c:%d\n",c);
    return c;
}

Eigen::Vector3d Pos;
Eigen::Vector3d  neural_bezier_curve::bezierCurve(double _Cpg,double phase)
{
    // MatrixXd kX,kY;
    // kX.resize(1,n);
    // kY.resize(1,n);
    // aX.resize(1,n);
    // aY.resize(1,n);
    // XCn.resize(1,n);
    // YCn.resize(1,n);

    // Xinput.resize(layerNum,N+1);
    // Xoutput.resize(layerNum,N+1);
    // Xoutput0.resize(1,N+1);
    // Yinput.resize(layerNum,N+1);
    // Youtput.resize(layerNum,N+1);
    // Youtput0.resize(1,N+1);
    // XCOM_BM.resize(1,N+1);
    // YCOM_BM.resize(1,N+1);

    // double Xinput_[layerNum]={0},Xinput_last[layerNum]={0};
    // double Xoutput_[layerNum]={0},Xoutput_last[layerNum]={0};
    // double Xoutput0_=0,Xoutput0_last[layerNum]={0};
    // double Yinput_[layerNum]={0},Yinput_last[layerNum]={0};
    // double Youtput_[layerNum]={0},Youtput_last[layerNum]={0};
    // double Youtput0_=0,Youtput0_last[layerNum]={0};
    // double XCOM_BM_=0,XCOM_BM_last[layerNum]={0};
    // double YCOM_BM_=0,YCOM_BM_last[layerNum]={0};

    // lamdaX.resize(1,n);
    // lamdaY.resize(1,n);

    setCurveHight(height_k);
    setCurveLength(length_k);


    for(int ni=0;ni<n;ni++)
    {   

        if(ni<=(n)/2 )
        {
            kX(ni)=1;   kY(ni)=1;//kX(1,ni)=1;        kY(1,ni)=1;    //%%kX中第一行第ni列的元素赋值为1 1*9的矩阵   
        }
        else
        {       
            kX(ni)=-1;   kY(ni)=-1;  
        }

        aX(ni)=0;        aY(ni)=0; // %%令a（i）=P（i）-P（i-1）的初始值为0

        if( ni==0||ni==n-1 )   //%%判断ni是否等于1还是n，有一真即为真
        {
            XCn(ni)=1;        YCn(ni)=1;
        }
        else
        {
            XCn(ni)=comb(layerNum,ni);   //%%Comb函数     
            YCn(ni)=comb(layerNum,ni);
        }

    }
    // std::cout<<XCn<<endl;
    // std::cout<<YCn<<endl;

    // for(double t=0;t<1;t=t+delaT) 
    // {

        t=0.5*_Cpg+0.5;
        TnIndex=TnIndex+1;
        WXout=pow(1-t,layerNum);//(1-t)^layerNum;
        wx=t/(1-t); //%连接权重
        WYout=pow(1-t,layerNum);
        wy=t/(1-t); //%连接权重
        // printf("TnIndex: %d ",TnIndex);
        // printf("N:%f\n",N);
        for(int i=0;i<layerNum;i++) //i=1:1:layerNum  % 第i层
        {
            

            //%%%%%%%%%% X 方向

            if(n-i-1>0)    //9 -0 -1   9-0
                aX(n-i-1)=PX[n-i-1]-PX[n-i-2];  //aX 8
            Xinput_[i]=PX[n-i-1-1]*XCn(n-i-1-1)+kX(n-i-1-1)*lamdaX(n-i-1-1)*aX(n-i-1-1)*(1-t);  

            if(i==0)
            {
                aX(n-1)=PX[n-1]-PX[n-1-1];
                Xoutput0_=PX[n-1]*XCn(n-1)+kX(n-1)*lamdaX(n-1)*aX(n-1)*(1-t);  //%%整个网络输入
                Xoutput_[i]=Xoutput0_*wx+Xinput_[i];//%%第1层输出
            }
            else
                Xoutput_[i]=Xoutput_[i-1]*wx+Xinput_[i];//%%第i层输出

            if(i==layerNum-1)
                Xoutput_[i]=Xoutput_[i]*WXout;


            //%%%%%%%%%% Y 方向
            if(n-i-1>0)    //9 -0 -1   9-0
                aY(n-i-1)=PY[n-i-1]-PY[n-i-2];  //aY 8
            Yinput_[i]=PY[n-i-1-1]*YCn(n-i-1-1)+kY(n-i-1-1)*lamdaY(n-i-1-1)*aY(n-i-1-1)*(1-t);  

            if(i==0)
            {
                aY(n-1)=PY[n-1]-PY[n-1-1];
                Youtput0_=PY[n-1]*YCn(n-1)+kY(n-1)*lamdaY(n-1)*aY(n-1)*(1-t);  //%%整个网络输入
                Youtput_[i]=Youtput0_*wy+Yinput_[i];//%%第1层输出
            }
            else
                Youtput_[i]=Youtput_[i-1]*wy+Yinput_[i];//%%第i层输出

            if(i==layerNum-1)
                Youtput_[i]=Youtput_[i]*WYout;


            // XCOM_BM(TnIndex-1)=Xoutput(i,TnIndex-1);//%躯干的x轴坐标
            // YCOM_BM(TnIndex-1)=Youtput(i,TnIndex-1);//%躯干的y轴坐标

            XCOM_BM_=Xoutput_[i];//%躯干的x轴坐标
            YCOM_BM_=Youtput_[i];//%躯干的y轴坐标
        }

    // }

    // Postion[0]=XCOM_BM(TnIndex-1);
    // Postion[1]=0;
    // Postion[2]=YCOM_BM(TnIndex-1);
    if( std::isinf(XCOM_BM_) || std::isnan(XCOM_BM_))  ;    else
        Postion[0]=XCOM_BM_*1;

    if( std::isinf(XCOM_BM_) || std::isnan(XCOM_BM_))   ;    else
        Postion[1]=0;

    if( std::isinf(YCOM_BM_) || std::isnan(YCOM_BM_))   ;    else
        Postion[2]=YCOM_BM_*phase*1;

    // printf("Postion[0]:%f   Postion[2]:%f   _Cpg:%f  phase:%f  XCOM_BM_:%f  YCOM_BM_:%f  \n"
                // ,Postion[0],Postion[2],_Cpg,phase,XCOM_BM_,YCOM_BM_);
    for (int i = 0; i < 3; i++)
    {
        Pos(i)=Postion[i];
    }
    

    return Pos;
}


// lcc yqr 20230427
// void neural_bezier_curve::yqrDL(void)
// {
//     MatrixXd F;
//     F.resize(1,N+1);

//     //     %%
//     // % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Dual Learner
//     // % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  2020.6.30
//     F=YCOM_BM;    
//     int ii=1;

//     double Af=1, Bf=0.01, As=1, Bs=0.002, trial=1000;  //%%%%%%%%%%%%%%%%%%%%%%%%%参数设置(最好的是Af设置为1)
//     MatrixXd FP, SP, LP, e, E;
//     FP.resize(n,N+1);
//     SP.resize(n,N+1);
//     LP.resize(n,N+1);
//     e.resize(1,N+1);
//     E.resize(1,N+1);

//     FP.setZero();
//     SP.setZero();
//     LP.setZero();
//     e.setZero();
//     E.setZero();

//     // int PYindex;
//     MatrixXd PY;
//     PY.resize(1,n);

//     // %%%%%%%%%%%%%%  LP为1X3的矩阵，而e同样转为1X3，故需要方法进行转化
//     for(int PYindex=0;PYindex<9;PYindex++)
//         PY(PYindex)=LP(PYindex,1);
//     // %%%%%%%%%% X 方向

 

//     Xinput.setZero();
//     Xoutput.setZero();
//     Xoutput0.setZero();

//     Yinput.setZero();
//     Youtput.setZero();
//     Youtput0.setZero();
    
//     for(int ni=0;ni<n;ni++)
//     {   

//         if(ni<=(n)/2 )
//         {
//             kX(ni)=1;   kY(ni)=1;//kX(1,ni)=1;        kY(1,ni)=1;    //%%kX中第一行第ni列的元素赋值为1 1*9的矩阵   
//         }
//         else
//         {       
//             kX(ni)=-1;   kY(ni)=-1;  
//         }

//         aX(ni)=0;        aY(ni)=0; // %%令a（i）=P（i）-P（i-1）的初始值为0

//         if( ni==0||ni==n-1 )   //%%判断ni是否等于1还是n，有一真即为真
//         {
//             XCn(ni)=1;        YCn(ni)=1;
//         }
//         else
//         {
//             XCn(ni)=comb(layerNum,ni);   //%%Comb函数     
//             YCn(ni)=comb(layerNum,ni);
//         }

//     }

//     MatrixXd eS, Lout;
//     eS.resize(n,1);
//     eS.setZero();
//     Lout.resize(1,N+1);
//     Lout.setZero();
//     for(int NN=0;NN<trial;NN++)
//     {
//         int TnIndex=0;
//         lamdaX.setZero();
//         lamdaY.setZero();

//         for (double t = 0; t < 1; t=t+0.005)
//         {
//             TnIndex=TnIndex+1;

//             Lout(TnIndex)=Youtput(ii,TnIndex);//%%Lout实际输出
//             e(TnIndex)=F(TnIndex)-Lout(TnIndex);

//             for (int Sindex = 0; Sindex < n; Sindex++)
//             {
//                 eS(Sindex)=YCn(Sindex)*pow(t,Sindex-1)*pow(1-t,n-Sindex);   //t;//^(Sindex-1)*(1-t)^(n-Sindex);
//             }
            
//             FP.block<9,1>(0,TnIndex+1)=Af*FP.block<9,1>(0,TnIndex)+Bf*e(TnIndex)*eS;
//             SP.block<9,1>(0,TnIndex+1)=As*SP.block<9,1>(0,TnIndex)+Bs*e(TnIndex)*eS;
//             LP.block<9,1>(0,TnIndex+1)=FP.block<9,1>(0,TnIndex+1)+SP.block<9,1>(0,TnIndex+1);
//             for(int PYindex=0;PYindex<9;PYindex++)
//                 PY(PYindex)=LP(PYindex,TnIndex+1);

//             XCOM_BM(0,TnIndex)=Xoutput(ii,TnIndex);//%躯干的x轴坐标
//             YCOM_BM(0,TnIndex)=Youtput(ii,TnIndex);//%躯干的y轴坐标
//             WXout=pow(1-t,layerNum);//(1-t)^layerNum;
//             wx=t/(1-t); //%连接权重
//             WYout=pow(1-t,layerNum);
//             wy=t/(1-t); //%连接权重
//             for(int i=0;i<n;i++)
//             {
//                 //%%%%%%%%%% X 方向
//                 if(n-i-1>0)    //9 -0 -1   9-0
//                     aX(n-i-1)=PX[n-i-1]-PX[n-i-2];  //aX 8

//                 Xinput_[TnIndex]=PX[n-i-1-1]*XCn(n-i-1-1)+kX(n-i-1-1)*lamdaX(n-i-1-1)*aX(n-i-1-1)*(1-t);  

//                 if(i==0)
//                 {
//                     aX(n-1)=PX[n-1]-PX[n-1-1];
//                     Xoutput0_=PX[n-1]*XCn(n-1)+kX(n-1)*lamdaX(n-1)*aX(n-1)*(1-t);  //%%整个网络输入
//                     Xoutput_[i]=Xoutput0_*wx+Xinput_[i];//%%第1层输出
//                 }
//                 else
//                     Xoutput_[i]=Xoutput_[i-1]*wx+Xinput_[i];//%%第i层输出

//                 if(i==layerNum-1)
//                     Xoutput_[i]=Xoutput_[i]*WXout;


//                 //%%%%%%%%%% Y 方向
//                 if(n-i-1>0)    //9 -0 -1   9-0
//                     aY(n-i-1)=PY[n-i-1]-PY[n-i-2];  //aY 8
//                 Yinput_[i]=PY[n-i-1-1]*YCn(n-i-1-1)+kY(n-i-1-1)*lamdaY(n-i-1-1)*aY(n-i-1-1)*(1-t);  

//                 if(i==0)
//                 {
//                     aY(n-1)=PY[n-1]-PY[n-1-1];
//                     Youtput0_=PY[n-1]*YCn(n-1)+kY(n-1)*lamdaY(n-1)*aY(n-1)*(1-t);  //%%整个网络输入
//                     Youtput_[i]=Youtput0_*wy+Yinput_[i];//%%第1层输出
//                 }
//                 else
//                     Youtput_[i]=Youtput_[i-1]*wy+Yinput_[i];//%%第i层输出

//                 if(i==layerNum-1)
//                     Youtput_[i]=Youtput_[i]*WYout;
//             }
//         }
//         FP.block<9,1>(0,0)=FP.block<9,1>(0,TnIndex);
//         SP.block<9,1>(0,0)=SP.block<9,1>(0,TnIndex);
//         LP.block<9,1>(0,0)=LP.block<9,1>(0,TnIndex);
//         e(0,0)=e(TnIndex-1);

//         for(int PYindex=0;PYindex<9;PYindex++)
//             PY(PYindex)=LP(PYindex,TnIndex+1);
//         E(NN)=e.sum();
        
//     }



// }
