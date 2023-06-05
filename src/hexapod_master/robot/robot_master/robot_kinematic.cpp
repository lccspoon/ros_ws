#include "robot_kinematic.h"
#include <math.h>

Eigen::Vector3d kinematic::invKinematic(Eigen::Vector3d Position_)
{  
    // Position[0]=0.3;Position[1]=10.5;Position[2]=-14;


    double invAngleTemp[3], Position[3];
    for (int i = 0; i < 3; i++)
    {
        Position[i]=Position_(i);
    }
    

    invAngleTemp[0]=atan2(Position[0],Position[1])-atan2(LEG_DH_PARAM1,pow((Position[1]*Position[1]+Position[0]*Position[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
    invAngleTemp[1]=acos((Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1+LEG_DH_PARAM2*LEG_DH_PARAM2-LEG_DH_PARAM3*LEG_DH_PARAM3)/(2*LEG_DH_PARAM2*pow((Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5)))
    -atan2(-Position[2],pow((Position[1]*Position[1]+Position[0]*Position[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
    invAngleTemp[2]=acos( (LEG_DH_PARAM2*LEG_DH_PARAM2+LEG_DH_PARAM3*LEG_DH_PARAM3-(Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1))/(2*LEG_DH_PARAM2*LEG_DH_PARAM3) )-3.1416;
    
    if( std::isinf(invAngleTemp[0]) || std::isnan(invAngleTemp[0]))  ;    else
        invAngle[0]=invAngleTemp[0];
    if( std::isinf(invAngleTemp[1]) || std::isnan(invAngleTemp[1]))  ;    else
        invAngle[1]=invAngleTemp[1];
    if( std::isinf(invAngleTemp[2]) || std::isnan(invAngleTemp[2]))  ;    else
        invAngle[2]=invAngleTemp[2];

    // std::cout<<"Des  "<<Position[0]<<" "<<Position[1]<<" "<<Position[2]<<std::endl;
    // std::cout<<"Ang  "<<invAngle[0]*180/3.1416<<" "<<invAngle[1]*180/3.1416<<" "<<invAngle[2]*180/3.1416<<std::endl;

    Eigen::Vector3d invANg;
    for (int i = 0; i < 3; i++)
    {
        invANg(i)=invAngle[i];
    }
    return invANg;
}

//20230518 zyt 足端修正
Eigen::Vector3d kinematic::CorrinvKinematic(Eigen::Vector3d Position_)
{
     double invAngleTemp[3],CorrinvAngleTemp[3], Position[3],corrPosition[3];
    for (int i = 0; i < 3; i++)
    {
        Position[i]=Position_(i);
        
    }
    
    invAngleTemp[0]=atan2(Position[0],Position[1])-atan2(LEG_DH_PARAM1,pow((Position[1]*Position[1]+Position[0]*Position[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
    invAngleTemp[1]=acos((Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1+LEG_DH_PARAM2*LEG_DH_PARAM2-LEG_DH_PARAM3*LEG_DH_PARAM3)/(2*LEG_DH_PARAM2*pow((Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5)))
    -atan2(-Position[2],pow((Position[1]*Position[1]+Position[0]*Position[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
    invAngleTemp[2]=acos( (LEG_DH_PARAM2*LEG_DH_PARAM2+LEG_DH_PARAM3*LEG_DH_PARAM3-(Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1))/(2*LEG_DH_PARAM2*LEG_DH_PARAM3) )-3.1416;
    
    
    // std::cout<<"Des  "<<Position[0]<<" "<<Position[1]<<" "<<Position[2]<<std::endl;
    // std::cout<<"Ang1  "<<invAngle[0]*180/3.1416<<" "<<invAngle[1]*180/3.1416<<" "<<invAngle[2]*180/3.1416<<std::endl;

    //修正参数
    fai=acos(-sin(invAngleTemp[1]+invAngleTemp[2]));
    datax=sin(invAngleTemp[0])*R*fai;
    datay=cos(invAngleTemp[0])*R*fai;
    dataz=0;

    corrPosition[0]=Position[0]+datax;
    corrPosition[1]=Position[1]+datay;
    corrPosition[2]=Position[2]+R;

    //修正后的关节角度
    CorrinvAngleTemp[0]=atan2(corrPosition[0],Position[1])-atan2(LEG_DH_PARAM1,pow((corrPosition[1]*corrPosition[1]+corrPosition[0]*corrPosition[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
    CorrinvAngleTemp[1]=acos((corrPosition[0]*Position[0]+corrPosition[1]*corrPosition[1]+corrPosition[2]*corrPosition[2]-LEG_DH_PARAM1*LEG_DH_PARAM1+LEG_DH_PARAM2*LEG_DH_PARAM2-l3*l3)/(2*LEG_DH_PARAM2*pow((corrPosition[0]*corrPosition[0]+corrPosition[1]*corrPosition[1]+corrPosition[2]*corrPosition[2]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5)))
    -atan2(-Position[2],pow((corrPosition[1]*corrPosition[1]+corrPosition[0]*corrPosition[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
    CorrinvAngleTemp[2]=acos( (LEG_DH_PARAM2*LEG_DH_PARAM2+l3*l3-(corrPosition[0]*corrPosition[0]+corrPosition[1]*corrPosition[1]+corrPosition[2]*corrPosition[2]-LEG_DH_PARAM1*LEG_DH_PARAM1))/(2*LEG_DH_PARAM2*l3) )-3.1416;

    for (int k=0;k<10;k++)
    {
        invAngleTemp[0]=CorrinvAngleTemp[0];
        invAngleTemp[1]=CorrinvAngleTemp[1];
        invAngleTemp[2]=CorrinvAngleTemp[2];

        fai=acos(-sin(invAngleTemp[1]+invAngleTemp[2]));
        datax=sin(invAngleTemp[0])*R*fai;
        datay=cos(invAngleTemp[0])*R*fai;
        dataz=0;
        
        corrPosition[0]=Position[0]+datax;
        corrPosition[1]=Position[1]+datay;
        corrPosition[2]=Position[2]+R;

        CorrinvAngleTemp[0]=atan2(corrPosition[0],corrPosition[1])-atan2(LEG_DH_PARAM1,pow((corrPosition[1]*corrPosition[1]+corrPosition[0]*corrPosition[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
        CorrinvAngleTemp[1]=acos((corrPosition[0]*corrPosition[0]+corrPosition[1]*corrPosition[1]+corrPosition[2]*corrPosition[2]-LEG_DH_PARAM1*LEG_DH_PARAM1+LEG_DH_PARAM2*LEG_DH_PARAM2-l3*l3)/(2*LEG_DH_PARAM2*pow((corrPosition[0]*corrPosition[0]+corrPosition[1]*corrPosition[1]+corrPosition[2]*corrPosition[2]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5)))
        -atan2(-corrPosition[2],pow((corrPosition[1]*corrPosition[1]+corrPosition[0]*corrPosition[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
        CorrinvAngleTemp[2]=acos( (LEG_DH_PARAM2*LEG_DH_PARAM2+l3*l3-(corrPosition[0]*corrPosition[0]+corrPosition[1]*corrPosition[1]+corrPosition[2]*corrPosition[2]-LEG_DH_PARAM1*LEG_DH_PARAM1))/(2*LEG_DH_PARAM2*l3) )-3.1416;

    }

    if( std::isinf(CorrinvAngleTemp[0]) || std::isnan(CorrinvAngleTemp[0]))  ;    else
        invAngle[0]=CorrinvAngleTemp[0];
    if( std::isinf(CorrinvAngleTemp[1]) || std::isnan(CorrinvAngleTemp[1]))  ;    else
        invAngle[1]=CorrinvAngleTemp[1];
    if( std::isinf(CorrinvAngleTemp[2]) || std::isnan(CorrinvAngleTemp[2]))  ;    else
        invAngle[2]=CorrinvAngleTemp[2];

    // std::cout<<"Des  "<<Position[0]<<" "<<Position[1]<<" "<<Position[2]<<std::endl;
    // std::cout<<"Ang  "<<CorrinvAngleTemp[0]*180/3.1416<<" "<<CorrinvAngleTemp[1]*180/3.1416<<" "<<CorrinvAngleTemp[2]*180/3.1416<<std::endl;

    Eigen::Vector3d invANg;
    for (int i = 0; i < 3; i++)
    {
        invANg(i)=invAngle[i];
    }
    return invANg;
}



void kinematic::showInfo(void)
{
    std::cout<<"forwardPosition  "<<forwardPosition[0]<<" "<<forwardPosition[1]<<" "<<forwardPosition[2]<<std::endl; 
    std::cout<<"IKAng  "<<invAngle[0]*180/3.1416<<" "<<invAngle[1]*180/3.1416<<" "<<invAngle[2]*180/3.1416<<std::endl; 
}

Eigen::Vector3d kinematic::forwardKinematic(Eigen::Vector3d angle_)
{
    double forwardPositionTemp[3];
    double angle[3];
    angle[0]=angle_(0);
    angle[1]=angle_(1);
    angle[2]=angle_(2);

    forwardPositionTemp[2]=-(-LEG_DH_PARAM3*sin((angle[1]+angle[2]))-LEG_DH_PARAM2*sin(angle[1]));
    forwardPositionTemp[0]=(LEG_DH_PARAM1*cos(angle[0])+LEG_DH_PARAM2*sin(angle[0])*cos(angle[1])+LEG_DH_PARAM3*sin(angle[0])*cos((angle[1]+angle[2])));
    forwardPositionTemp[1]=-(LEG_DH_PARAM1*sin(angle[0])-LEG_DH_PARAM2*cos(angle[0])*cos(angle[1])-LEG_DH_PARAM3*cos(angle[0])*cos((angle[1]+angle[2])));
    
    if( std::isinf(forwardPositionTemp[0]) || std::isnan(forwardPositionTemp[0]))  ;    else
        forwardPosition[0]=forwardPositionTemp[0];
    if( std::isinf(forwardPositionTemp[1]) || std::isnan(forwardPositionTemp[1]))  ;    else
        forwardPosition[1]=forwardPositionTemp[1];
    if( std::isinf(forwardPositionTemp[2]) || std::isnan(forwardPositionTemp[2]))  ;    else
        forwardPosition[2]=forwardPositionTemp[2];

    // std::cout<<"ang  "<<angle[0]<<" "<<angle[1]<<" "<<angle[2]<<std::endl;
    // std::cout<<"For  "<<forwardPosition[0]<<" "<<forwardPosition[1]<<" "<<forwardPosition[2]<<std::endl;

    Eigen::Vector3d ppp;
    ppp<<forwardPosition[0],forwardPosition[1],forwardPosition[2];

    return ppp;
}

// Eigen::Matrix<double,3,3> kinematic::jacobiTranInv(char **leg_name,double angle[0],double angle[1],double angle[2])
Eigen::Matrix<double,3,3> kinematic::jacobiTranInv(int leg_number,Eigen::Vector3d angle_)
{

    // std::cout<<*leg_name<<std::endl;
    if(leg_number==1 || leg_number==2 || leg_number==3)  // 1->lf  2->lm  3->lb
    {
        Jacobian<<
                LEG_DH_PARAM3*cos(angle_(1) + angle_(2))*cos(angle_(0)) - LEG_DH_PARAM1*sin(angle_(0)) + LEG_DH_PARAM2*cos(angle_(0))*cos(angle_(1)),
                - LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*sin(angle_(0)) - LEG_DH_PARAM2*sin(angle_(0))*sin(angle_(1)),
                -LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*sin(angle_(0)),

                - LEG_DH_PARAM1*cos(angle_(0)) - LEG_DH_PARAM3*cos(angle_(1) + angle_(2))*sin(angle_(0)) - LEG_DH_PARAM2*cos(angle_(1))*sin(angle_(0)),
                - LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*cos(angle_(0)) - LEG_DH_PARAM2*cos(angle_(0))*sin(angle_(1)),
                -LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*cos(angle_(0))   ,

                0,                           
                LEG_DH_PARAM3*cos(angle_(1) + angle_(2)) + LEG_DH_PARAM2*cos(angle_(1)),              
                LEG_DH_PARAM3*cos(angle_(1) + angle_(2));
    }
    else
    {
        Jacobian<<
                -( LEG_DH_PARAM3*cos(angle_(1) + angle_(2))*cos(angle_(0)) - LEG_DH_PARAM1*sin(angle_(0)) + LEG_DH_PARAM2*cos(angle_(0))*cos(angle_(1)) ),
                -( - LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*sin(angle_(0)) - LEG_DH_PARAM2*sin(angle_(0))*sin(angle_(1)) ),
                -( -LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*sin(angle_(0)) ),

                ( - LEG_DH_PARAM1*cos(angle_(0)) - LEG_DH_PARAM3*cos(angle_(1) + angle_(2))*sin(angle_(0)) - LEG_DH_PARAM2*cos(angle_(1))*sin(angle_(0)) ),
                ( - LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*cos(angle_(0)) - LEG_DH_PARAM2*cos(angle_(0))*sin(angle_(1)) ),
                ( -LEG_DH_PARAM3*sin(angle_(1) + angle_(2))*cos(angle_(0))   ),

                -0,                           
                -( LEG_DH_PARAM3*cos(angle_(1) + angle_(2)) + LEG_DH_PARAM2*cos(angle_(1)) ),              
                -( LEG_DH_PARAM3*cos(angle_(1) + angle_(2)) );
    }
    // cout<<"Jacobian"<<Jacobian<<endl;
    Jacobian_T=Jacobian.transpose();
    // cout<<"Jacobian_T"<<Jacobian_T<<endl;
    Jacobian_T_inv=Jacobian_T.inverse();
    //cout<<Jacobian_T_inv<<endl;
    return Jacobian_T_inv;
}

Eigen::Matrix<double,3,3> kinematic::jacobi(void)
{
    return Jacobian;
}