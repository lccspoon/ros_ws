#include "robot_utilities.h"
#include "ros/ros.h"
void Hexapod::msgShow(void)
{
        // cout << "cost time:" << dt_ms <<"ms     "<< "rec_data_seq" << rec_data_seq << endl;

        // std::cout<<"leg_root.foot_des_pos"<<std::endl;
        // std::cout<<leg_root.foot_des_pos<<std::endl;
        // std::cout<<"leg_root.foot_act_pos"<<std::endl;
        // std::cout<<leg_root.foot_act_pos<<std::endl;

        // std::cout<<"body_root.foot_des_pos"<<std::endl;
        // std::cout<<body_root.foot_des_pos<<std::endl;
        // std::cout<<"body_root.foot_act_pos"<<std::endl;
        // std::cout<<body_root.foot_act_pos<<std::endl;

        // std::cout<<"world_root.body_des_vel"<<std::endl;
        // std::cout<<world_root.body_des_vel<<std::endl;
        // std::cout<<"world_root.body_des_pos"<<std::endl;
        // std::cout<<world_root.body_des_pos<<std::endl;

        // std::cout<<"world_root.foot_des_pos"<<std::endl;
        // std::cout<<world_root.foot_des_pos<<std::endl;

        // std::cout<<"state.root_pos"<<std::endl;
        // std::cout<<state.root_pos<<std::endl;
        // std::cout<<"state.root_lin_vel"<<std::endl;
        // std::cout<<state.root_lin_vel<<std::endl;
        // std::cout<<"state.foot_pos_ref_world"<<std::endl;
        // std::cout<<state.foot_pos_ref_world<<std::endl;

        // printf("\n -------next-----\n");
}

/**
* @details 1.接收关节信息
* @details 2.基于时间序列是否更新来阻塞，这个功能可根据宏SEQ_CHOKE来开启or关闭
* @details 3.调用recJointData()接收gazebo或机器人在数据，存储在 joi_act_vel，joi_act_tor中; 
* @details 4.判断是否接收到关节信息，标志位存在get_joint_pos_flag中; 若是真实机器人，会限制上电位置;
* @details 5．接收其他传感消息;
* @author lcc
*/
std::mutex mtx2;int mtx2_flag=0;
void Hexapod::recData()
{       
        #if HARD_WARE==1
                rec_data_seq=HexInt.data_sequence=HarBri.output_sequnce; 
                #if SEQ_CHOKE==1
                        while(rec_data_seq==rec_data_seq_last)
                        {       
                                HarBri.dataRecAllAndSeqUpdate();
                                // printf("die in Hexapod::recData() !!! rec_data_seq:%d rec_data_seq_last:%d \n",rec_data_seq,rec_data_seq_last);
                                rec_data_seq=HexInt.data_sequence=HarBri.output_sequnce;
                        }
                #endif
                // printf("rec_data_seq:%d rec_data_seq_last:%d\n",rec_data_seq,rec_data_seq_last);
                rec_data_seq_last=rec_data_seq;
                HarBri.dataRecAllAndSeqUpdate();
		HarBri.dataIntegrat();

                // //xzb230522//////////////////////////////
                // body_root.imu_ang_vel=
                // body_root.imu_lin_acc =
                // body_root.imu_qua_ori=

                // Eigen::Quaterniond q(body_root.imu_qua_ori(3), body_root.imu_qua_ori(0), body_root.imu_qua_ori(1), body_root.imu_qua_ori(2));
                // q=q.normalized();
                // body_root.root_rot_mat=q.toRotationMatrix();

                // // 4.4 四元数转欧拉角(Z-Y-X，即RPY)
                // body_root.eulerAngle=q.matrix().eulerAngles(2,1,0);
                // for (int i = 0; i < 3; i++)
                // {
                //         if(  body_root.eulerAngle(i) * _RAD2  >=90 )  //lcc 20230518:注意，这样写会有ｂｕｇ，导致大于９０度会判断错误．先不管，以后改
                //         {
                //                 body_root.eulerAngle(i)=3.14-body_root.eulerAngle(i);
                //         }
                //         else if(  body_root.eulerAngle(i) * _RAD2  <=-90 )
                //         {
                //                 body_root.eulerAngle(i)= - ( 3.14+body_root.eulerAngle(i) );
                //         }
                // }
                // //xzb230522//////////////////////////////
        #elif HARD_WARE==2
                rec_data_seq=HexInt.data_sequence=GazeboSim.retLegMsgSeq();
                #if SEQ_CHOKE==1
                        while(rec_data_seq==rec_data_seq_last)
                        {       
                                // printf("die in Hexapod::recData()  !!! rec_data_seq:%d rec_data_seq_last:%d \n",rec_data_seq,rec_data_seq_last);
                                rec_data_seq=HexInt.data_sequence=SIM_SEQ;
                                // GazeboSim.retLegMsgSeq();
                        }
                        rec_data_seq_last=rec_data_seq;
                #elif SEQ_CHOKE==2
                        ros::spinOnce();
                        rec_data_seq=HexInt.data_sequence=SIM_SEQ;
                #endif

                body_root.imu_ang_vel=GazeboSim.retImuAngVel();
                body_root.imu_lin_acc =GazeboSim.retImuLinAcc();
                body_root.imu_qua_ori=GazeboSim.retImuQuaOri();

                Eigen::Quaterniond q(body_root.imu_qua_ori(3), body_root.imu_qua_ori(0), body_root.imu_qua_ori(1), body_root.imu_qua_ori(2));
                q=q.normalized();
                body_root.root_rot_mat=q.toRotationMatrix();

                // 4.4 四元数转欧拉角(Z-Y-X，即RPY)
                body_root.eulerAngle=q.matrix().eulerAngles(2,1,0);
                for (int i = 0; i < 3; i++)
                {
                        if(  body_root.eulerAngle(i) * _RAD2  >=90 )  //lcc 20230518:注意，这样写会有ｂｕｇ，导致大于９０度会判断错误．先不管，以后改
                        {
                                body_root.eulerAngle(i)=3.14-body_root.eulerAngle(i);
                        }
                        else if(  body_root.eulerAngle(i) * _RAD2  <=-90 )
                        {
                                body_root.eulerAngle(i)= - ( 3.14+body_root.eulerAngle(i) );
                        }
                }
        #endif
        
        /**
        * @brief 接收gazebo或机器人在数据，存储在 joi_act_vel，joi_act_tor中
        * @author lcc
        */
        #if HARD_WARE==1
                leg_root.joint_rec_pos=HarBri.retWhloePos();
                leg_root.joint_rec_vel=HarBri.retWhloeVel();
                leg_root.joint_rec_tor=HarBri.retWhloeTor();   

        #elif HARD_WARE==2
                leg_root.joint_rec_pos=GazeboSim.retLegMsgPosition();
                leg_root.joint_rec_vel=GazeboSim.retLegMsgVelocity();
                leg_root.joint_rec_tor=GazeboSim.retLegMsgTorque();

                leg_root.foot_ret_force=GazeboSim.retFootEndForec();
                leg_root.joint_rec_pos(9)=-leg_root.joint_rec_pos(9);leg_root.joint_rec_pos(10)=-leg_root.joint_rec_pos(10);leg_root.joint_rec_pos(11)=-leg_root.joint_rec_pos(11);
                leg_root.joint_rec_pos(12)=-leg_root.joint_rec_pos(12);leg_root.joint_rec_pos(13)=-leg_root.joint_rec_pos(13);leg_root.joint_rec_pos(14)=-leg_root.joint_rec_pos(14);
                leg_root.joint_rec_pos(15)=-leg_root.joint_rec_pos(15);leg_root.joint_rec_pos(16)=-leg_root.joint_rec_pos(16);leg_root.joint_rec_pos(17)=-leg_root.joint_rec_pos(17);
        #endif

}

void Hexapod::recDataHandling(void)
{
        //机器坐标系方向跟据右手定则： 向前为x正，向左为y正，向上为z正。
        //lcc 20230513: 跟关节下真实足端位置
        #if HARD_WARE==1
                leg_root.foot_act_pos.block<3,1>(0,5)=_kinematic[5].forwardKinematic(leg_root.joint_rec_pos.block<1,3>(0,15).transpose()-HarBri.rb_off_set);
                leg_root.foot_act_pos.block<3,1>(0,4)=_kinematic[4].forwardKinematic(leg_root.joint_rec_pos.block<1,3>(0,12).transpose()-HarBri.rm_off_set);
                leg_root.foot_act_pos.block<3,1>(0,3)=_kinematic[3].forwardKinematic(leg_root.joint_rec_pos.block<1,3>(0,9).transpose()-HarBri.rf_off_set);
                leg_root.foot_act_pos.block<3,1>(0,2)=_kinematic[2].forwardKinematic(-leg_root.joint_rec_pos.block<1,3>(0,6).transpose()+HarBri.lb_off_set);
                leg_root.foot_act_pos.block<3,1>(0,1)=_kinematic[1].forwardKinematic(-leg_root.joint_rec_pos.block<1,3>(0,3).transpose()+HarBri.lm_off_set);
                leg_root.foot_act_pos.block<3,1>(0,0)=_kinematic[0].forwardKinematic(-leg_root.joint_rec_pos.block<1,3>(0,0).transpose()+HarBri.lf_off_set);
        #elif HARD_WARE==2
                leg_root.foot_act_pos.block<3,1>(0,5)=_kinematic[5].forwardKinematic(leg_root.joint_rec_pos.block<1,3>(0,15).transpose());
                leg_root.foot_act_pos.block<3,1>(0,4)=_kinematic[4].forwardKinematic(leg_root.joint_rec_pos.block<1,3>(0,12).transpose());
                leg_root.foot_act_pos.block<3,1>(0,3)=_kinematic[3].forwardKinematic(leg_root.joint_rec_pos.block<1,3>(0,9).transpose());
                leg_root.foot_act_pos.block<3,1>(0,2)=_kinematic[2].forwardKinematic(leg_root.joint_rec_pos.block<1,3>(0,6).transpose());
                leg_root.foot_act_pos.block<3,1>(0,1)=_kinematic[1].forwardKinematic(leg_root.joint_rec_pos.block<1,3>(0,3).transpose());
                leg_root.foot_act_pos.block<3,1>(0,0)=_kinematic[0].forwardKinematic(leg_root.joint_rec_pos.block<1,3>(0,0).transpose());
        #endif
        leg_root.foot_act_pos.block<1,1>(1,5)=-leg_root.foot_act_pos.block<1,1>(1,5);
        leg_root.foot_act_pos.block<1,1>(1,4)=-leg_root.foot_act_pos.block<1,1>(1,4);
        leg_root.foot_act_pos.block<1,1>(1,3)=-leg_root.foot_act_pos.block<1,1>(1,3);

        //lcc 20230513: 跟关节系相对于机身系的位置
        body_root.leg_origin.block<3,1>(0,5)=_FooBodAdjMap[5].legOriRefToBody();
        body_root.leg_origin.block<3,1>(0,4)=_FooBodAdjMap[4].legOriRefToBody();
        body_root.leg_origin.block<3,1>(0,3)=_FooBodAdjMap[3].legOriRefToBody();
        body_root.leg_origin.block<3,1>(0,2)=_FooBodAdjMap[2].legOriRefToBody();
        body_root.leg_origin.block<3,1>(0,1)=_FooBodAdjMap[1].legOriRefToBody();
        body_root.leg_origin.block<3,1>(0,0)=_FooBodAdjMap[0].legOriRefToBody();
        // std::cout<<"body_root.leg_origin: "<<std::endl;
        // std::cout<<body_root.leg_origin<<std::endl;

        //lcc 20230513: 机身系下真足端实位置
        body_root.foot_act_pos.block<3,1>(0,5)=leg_root.foot_act_pos.block<3,1>(0,5)+body_root.leg_origin.block<3,1>(0,5);
        body_root.foot_act_pos.block<3,1>(0,4)=leg_root.foot_act_pos.block<3,1>(0,4)+body_root.leg_origin.block<3,1>(0,4);
        body_root.foot_act_pos.block<3,1>(0,3)=leg_root.foot_act_pos.block<3,1>(0,3)+body_root.leg_origin.block<3,1>(0,3);
        body_root.foot_act_pos.block<3,1>(0,2)=leg_root.foot_act_pos.block<3,1>(0,2)+body_root.leg_origin.block<3,1>(0,2);
        body_root.foot_act_pos.block<3,1>(0,1)=leg_root.foot_act_pos.block<3,1>(0,1)+body_root.leg_origin.block<3,1>(0,1);
        body_root.foot_act_pos.block<3,1>(0,0)=leg_root.foot_act_pos.block<3,1>(0,0)+body_root.leg_origin.block<3,1>(0,0);

        // lcc 20230513: 机身系下期望足端位置
        #if HARD_WARE==1
                leg_root.foot_des_pos.block<3,1>(0,5)=_kinematic[5].forwardKinematic(leg_root.joint_des_pos.block<3,1>(0,5)-HarBri.rb_off_set);
                leg_root.foot_des_pos.block<3,1>(0,4)=_kinematic[4].forwardKinematic(leg_root.joint_des_pos.block<3,1>(0,4)-HarBri.rm_off_set);
                leg_root.foot_des_pos.block<3,1>(0,3)=_kinematic[3].forwardKinematic(leg_root.joint_des_pos.block<3,1>(0,3)-HarBri.rf_off_set);
                leg_root.foot_des_pos.block<3,1>(0,2)=_kinematic[2].forwardKinematic(-leg_root.joint_des_pos.block<3,1>(0,2)+HarBri.lb_off_set);
                leg_root.foot_des_pos.block<3,1>(0,1)=_kinematic[1].forwardKinematic(-leg_root.joint_des_pos.block<3,1>(0,1)+HarBri.lm_off_set);
                leg_root.foot_des_pos.block<3,1>(0,0)=_kinematic[0].forwardKinematic(-leg_root.joint_des_pos.block<3,1>(0,0)+HarBri.lf_off_set);
        #else HARD_WARE==2
                leg_root.foot_des_pos.block<3,1>(0,5)=_kinematic[5].forwardKinematic(leg_root.joint_des_pos.block<3,1>(0,5));
                leg_root.foot_des_pos.block<3,1>(0,4)=_kinematic[4].forwardKinematic(leg_root.joint_des_pos.block<3,1>(0,4));
                leg_root.foot_des_pos.block<3,1>(0,3)=_kinematic[3].forwardKinematic(leg_root.joint_des_pos.block<3,1>(0,3));
                leg_root.foot_des_pos.block<3,1>(0,2)=_kinematic[2].forwardKinematic(leg_root.joint_des_pos.block<3,1>(0,2));
                leg_root.foot_des_pos.block<3,1>(0,1)=_kinematic[1].forwardKinematic(leg_root.joint_des_pos.block<3,1>(0,1));
                leg_root.foot_des_pos.block<3,1>(0,0)=_kinematic[0].forwardKinematic(leg_root.joint_des_pos.block<3,1>(0,0));
        #endif
        leg_root.foot_des_pos.block<1,1>(1,5)=-leg_root.foot_des_pos.block<1,1>(1,5);
        leg_root.foot_des_pos.block<1,1>(1,4)=-leg_root.foot_des_pos.block<1,1>(1,4);
        leg_root.foot_des_pos.block<1,1>(1,3)=-leg_root.foot_des_pos.block<1,1>(1,3);

        body_root.foot_des_pos.block<3,1>(0,5)=leg_root.foot_des_pos.block<3,1>(0,5)+body_root.leg_origin.block<3,1>(0,5);
        body_root.foot_des_pos.block<3,1>(0,4)=leg_root.foot_des_pos.block<3,1>(0,4)+body_root.leg_origin.block<3,1>(0,4);
        body_root.foot_des_pos.block<3,1>(0,3)=leg_root.foot_des_pos.block<3,1>(0,3)+body_root.leg_origin.block<3,1>(0,3);
        body_root.foot_des_pos.block<3,1>(0,2)=leg_root.foot_des_pos.block<3,1>(0,2)+body_root.leg_origin.block<3,1>(0,2);
        body_root.foot_des_pos.block<3,1>(0,1)=leg_root.foot_des_pos.block<3,1>(0,1)+body_root.leg_origin.block<3,1>(0,1);
        body_root.foot_des_pos.block<3,1>(0,0)=leg_root.foot_des_pos.block<3,1>(0,0)+body_root.leg_origin.block<3,1>(0,0);

        // lcc 20230513: 足端力(计算公式：F= (j_T)inv * tor );jacobiTranInv()中还计算了雅克比矩阵  
        //               20230504:ok
        #if HARD_WARE==1
                //lcc 20230521:足端力的方向和大小很不可靠！！！建议用合力就行，无法判断方向！
                leg_root.foot_act_force.block<3,1>(0,0)=_kinematic[0].jacobiTranInv(0,-leg_root.joint_rec_pos.block<1,3>(0,0).transpose()+HarBri.lf_off_set)*leg_root.joint_rec_tor.block<1,3>(0,0).transpose();
                // leg_root.foot_act_force.block<3,1>(0,0)=-leg_root.foot_act_force.block<3,1>(0,0);
                leg_root.foot_act_force.block<3,1>(0,1)=_kinematic[1].jacobiTranInv(1,-leg_root.joint_rec_pos.block<1,3>(0,1).transpose()+HarBri.lm_off_set)*leg_root.joint_rec_tor.block<1,3>(0,3).transpose();
                // leg_root.foot_act_force(2,1)=-leg_root.foot_act_force(2,1);
                leg_root.foot_act_force.block<3,1>(0,2)=_kinematic[2].jacobiTranInv(2,-leg_root.joint_rec_pos.block<1,3>(0,2).transpose()+HarBri.lb_off_set)*leg_root.joint_rec_tor.block<1,3>(0,6).transpose();
                leg_root.foot_act_force.block<3,1>(0,3)=_kinematic[3].jacobiTranInv(3,leg_root.joint_rec_pos.block<1,3>(0,3).transpose()-HarBri.rf_off_set)*leg_root.joint_rec_tor.block<1,3>(0,9).transpose();
                leg_root.foot_act_force.block<3,1>(0,4)=_kinematic[4].jacobiTranInv(4,leg_root.joint_rec_pos.block<1,3>(0,4).transpose()-HarBri.rm_off_set)*leg_root.joint_rec_tor.block<1,3>(0,12).transpose();
                leg_root.foot_act_force.block<3,1>(0,5)=_kinematic[5].jacobiTranInv(5,leg_root.joint_rec_pos.block<1,3>(0,5).transpose()-HarBri.rb_off_set)*leg_root.joint_rec_tor.block<1,3>(0,15).transpose();
                // std::cout<<"leg_root.foot_act_force.block<3,1>(0,2)"<<std::endl;
                // std::cout<<leg_root.foot_act_force.block<3,1>(0,2)<<std::endl;
                // std::cout<<"next"<<std::endl;
        #else HARD_WARE==2
                leg_root.foot_act_force.block<3,1>(0,0)=_kinematic[0].jacobiTranInv(0,leg_root.joint_rec_pos.block<1,3>(0,0).transpose())*leg_root.joint_rec_tor.block<1,3>(0,0).transpose();
                leg_root.foot_act_force.block<3,1>(0,1)=_kinematic[1].jacobiTranInv(1,leg_root.joint_rec_pos.block<1,3>(0,1).transpose())*leg_root.joint_rec_tor.block<1,3>(0,3).transpose();
                leg_root.foot_act_force.block<3,1>(0,2)=_kinematic[2].jacobiTranInv(2,leg_root.joint_rec_pos.block<1,3>(0,2).transpose())*leg_root.joint_rec_tor.block<1,3>(0,6).transpose();
                leg_root.foot_act_force.block<3,1>(0,3)=_kinematic[3].jacobiTranInv(3,leg_root.joint_rec_pos.block<1,3>(0,3).transpose())*leg_root.joint_rec_tor.block<1,3>(0,9).transpose();
                leg_root.foot_act_force.block<3,1>(0,4)=_kinematic[4].jacobiTranInv(4,leg_root.joint_rec_pos.block<1,3>(0,4).transpose())*leg_root.joint_rec_tor.block<1,3>(0,12).transpose();
                leg_root.foot_act_force.block<3,1>(0,5)=_kinematic[5].jacobiTranInv(5,leg_root.joint_rec_pos.block<1,3>(0,5).transpose())*leg_root.joint_rec_tor.block<1,3>(0,15).transpose();
        #endif

        // lcc 20230513: 足端速度(计算公式：x_dot=j*q_dot);  
        //               修改雅克比矩阵可以修改速度值正负号来校正方向;
        //               仿真中已经验证过，是正确的;
        #if HARD_WARE==1
        body_root.foot_act_vel.block<3,1>(0,0)=_kinematic[0].jacobi()*(-leg_root.joint_rec_vel.block<1,3>(0,0).transpose());
        body_root.foot_act_vel.block<3,1>(0,1)=_kinematic[1].jacobi()*(leg_root.joint_rec_vel.block<1,3>(0,3).transpose());
        body_root.foot_act_vel(1,1)=-body_root.foot_act_vel(1,1);
        body_root.foot_act_vel.block<3,1>(0,2)=_kinematic[2].jacobi()*(leg_root.joint_rec_vel.block<1,3>(0,6).transpose());
        body_root.foot_act_vel(1,2)=-body_root.foot_act_vel(1,2);
        body_root.foot_act_vel(2,2)=-body_root.foot_act_vel(2,2);
        body_root.foot_act_vel.block<3,1>(0,3)=_kinematic[3].jacobi()*(leg_root.joint_rec_vel.block<1,3>(0,9).transpose());
        body_root.foot_act_vel(2,3)=-body_root.foot_act_vel(2,3);
        body_root.foot_act_vel.block<3,1>(0,4)=_kinematic[4].jacobi()*(leg_root.joint_rec_vel.block<1,3>(0,12).transpose());
        body_root.foot_act_vel.block<3,1>(0,5)=_kinematic[5].jacobi()*(leg_root.joint_rec_vel.block<1,3>(0,15).transpose());
        body_root.foot_act_vel(1,5)=-body_root.foot_act_vel(1,5);
        body_root.foot_act_vel(2,5)=-body_root.foot_act_vel(2,5);
        #else HARD_WARE==2

        #endif
}

void Hexapod::simMsgPub(void)
{
        #if HARD_WARE==2   
                if(get_joint_pos_flag==false 
                && leg_root.joint_rec_pos(0)!=0 && leg_root.joint_rec_pos(1)!=0 && leg_root.joint_rec_pos(2)!=0 
                && leg_root.joint_rec_pos(3)!=0 && leg_root.joint_rec_pos(4)!=0 && leg_root.joint_rec_pos(5)!=0 
                && leg_root.joint_rec_pos(6)!=0 && leg_root.joint_rec_pos(7)!=0 && leg_root.joint_rec_pos(8)!=0 
                && leg_root.joint_rec_pos(9)!=0 && leg_root.joint_rec_pos(10)!=0 && leg_root.joint_rec_pos(11)!=0 
                && leg_root.joint_rec_pos(12)!=0 && leg_root.joint_rec_pos(13)!=0 && leg_root.joint_rec_pos(14)!=0 
                && leg_root.joint_rec_pos(15)!=0 && leg_root.joint_rec_pos(16)!=0 && leg_root.joint_rec_pos(17)!=0 
                )
                {
                        for(int i=0;i<6;i++)
                        {
                                leg_root.joint_des_pos.block<3,1>(0,i) =leg_root.joint_rec_pos.block<1,3>(0,i*3);
                        }
                        printf("\n------------------------------!!!!!!---------------------------------------\n");
                        std::cout<<"leg_root.joint_des_pos"<<std::endl;
                        std::cout<<leg_root.joint_des_pos*_RAD2<<std::endl;
                        get_joint_pos_flag=true;    //lcc 20230405:    如果接收到了电机的当前位置点书籍，那么motor_pos_get_flag=true;                      
                }
                if(joint_pos_run_count>=3) // lcc 20230405: 表示已经从机器人起始点角度向目标角度以等差逼近的方式，逼近了两步
                        {
                                #if SIM_PROTECT==1
                                        _dataUnuProtect[0].velLimAndDifFroDesPosAndActPos(0, 3,
                                                                                        leg_root.joint_des_pos.block<3, 1>(0, 0), 
                                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 0).transpose(), 20 * _RAD1,
                                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 0).transpose(), 9);
                                        _dataUnuProtect[1].velLimAndDifFroDesPosAndActPos(1, 3,
                                                                                        leg_root.joint_des_pos.block<3, 1>(0, 1), 
                                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 3).transpose(), 20 * _RAD1,
                                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 3).transpose(), 9);
                                        _dataUnuProtect[2].velLimAndDifFroDesPosAndActPos(2, 3,
                                                                                        leg_root.joint_des_pos.block<3, 1>(0, 2), 
                                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 6).transpose(), 20 * _RAD1,
                                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 6).transpose(), 9);

                                        _dataUnuProtect[3].velLimAndDifFroDesPosAndActPos(3, 3,
                                                                                        leg_root.joint_des_pos.block<3, 1>(0, 3), 
                                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 9).transpose(), 20 * _RAD1,
                                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 9).transpose(), 9);
                                        _dataUnuProtect[4].velLimAndDifFroDesPosAndActPos(4, 3,
                                                                                        leg_root.joint_des_pos.block<3, 1>(0, 4), 
                                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 12).transpose(), 20 * _RAD1,
                                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 12).transpose(), 9);
                                        _dataUnuProtect[5].velLimAndDifFroDesPosAndActPos(5, 3,
                                                                                        leg_root.joint_des_pos.block<3, 1>(0, 5), 
                                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 15).transpose(), 20 * _RAD1,
                                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 15).transpose(), 9);
                                #endif
                                if(_dataUnuProtect[5].diff_val_flag==false or _dataUnuProtect[4].diff_val_flag==false 
                                or _dataUnuProtect[3].diff_val_flag==false or _dataUnuProtect[2].diff_val_flag==false 
                                or _dataUnuProtect[1].diff_val_flag==false or _dataUnuProtect[0].diff_val_flag==false)
                                {
                                        _dataUnuProtect[5].diff_val_flag=false;_dataUnuProtect[4].diff_val_flag=false;
                                        _dataUnuProtect[3].diff_val_flag=false;_dataUnuProtect[2].diff_val_flag=false;
                                        _dataUnuProtect[1].diff_val_flag=false;_dataUnuProtect[0].diff_val_flag=false;
                                        printf("\n   ------------diff_val_flag:%d --------------\n",_dataUnuProtect[5].diff_val_flag);
                                }
                                else if(_dataUnuProtect[5].vel_lim_flag==false or _dataUnuProtect[4].vel_lim_flag==false 
                                or _dataUnuProtect[3].vel_lim_flag==false or _dataUnuProtect[2].vel_lim_flag==false 
                                or _dataUnuProtect[1].vel_lim_flag==false or _dataUnuProtect[0].vel_lim_flag==false)
                                {
                                        _dataUnuProtect[5].vel_lim_flag=false;_dataUnuProtect[4].vel_lim_flag=false;
                                        _dataUnuProtect[3].vel_lim_flag=false;_dataUnuProtect[2].vel_lim_flag=false;
                                        _dataUnuProtect[1].vel_lim_flag=false;_dataUnuProtect[0].vel_lim_flag=false;
                                        printf("\n   ------------vel_lim_flag:%d --------------\n",_dataUnuProtect[5].vel_lim_flag);
                                }
                                else
                                {
                                        GazeboSim.legMsgPub(leg_root.joint_msg_pub);
                                }
                        }
                        // usleep(3000);// lcc 20230405: 因为发送控制位置后电机还不能立刻到达，会导致实际角度和期望角度相差过大(详见函数differFromDesPosAndActPos)。所以此处加延时，让电机有反应的时间。
        #endif
}


/**
* @details 1.在recData()中接收到真实关节位置后，get_joint_pos_flag=true后,
*               joint_pos_run_count会在trajectoryPlaning()中进行累加;
* @details 2.通过velLimAnddifFroDesPosAndActPos()对joi_act_vel、joi_act_pos与joi_des_pos在差值进行保护; 
* @details 3.最后调用realHexMsgLoadAndSend()来发送电机的控制; 
*/
void Hexapod::realRobMsgPub(void)
{

        if(mtx2_flag==0)
        {
                // mtx2.lock();
                if(get_joint_pos_flag==false 
                && leg_root.joint_rec_pos(0)!=0 && leg_root.joint_rec_pos(1)!=0 && leg_root.joint_rec_pos(2)!=0 
                && leg_root.joint_rec_pos(3)!=0 && leg_root.joint_rec_pos(4)!=0 && leg_root.joint_rec_pos(5)!=0 
                && leg_root.joint_rec_pos(6)!=0 && leg_root.joint_rec_pos(7)!=0 && leg_root.joint_rec_pos(8)!=0 
                && leg_root.joint_rec_pos(9)!=0 && leg_root.joint_rec_pos(10)!=0 && leg_root.joint_rec_pos(11)!=0 
                && leg_root.joint_rec_pos(12)!=0 && leg_root.joint_rec_pos(13)!=0 && leg_root.joint_rec_pos(14)!=0 
                && leg_root.joint_rec_pos(15)!=0 && leg_root.joint_rec_pos(16)!=0 && leg_root.joint_rec_pos(17)!=0 
                //lcc 20230412:对机器人的上电位置进行限制
                && fabs(leg_root.joint_rec_pos(0))<=90*_RAD1 && fabs(leg_root.joint_rec_pos(1))<=90*_RAD1 &&fabs(leg_root.joint_rec_pos(2))<=100*_RAD1
                && fabs(leg_root.joint_rec_pos(3))<=90*_RAD1 && fabs(leg_root.joint_rec_pos(4))<=90*_RAD1 &&fabs(leg_root.joint_rec_pos(5))<=100*_RAD1
                && fabs(leg_root.joint_rec_pos(6))<=90*_RAD1 && fabs(leg_root.joint_rec_pos(7))<=90*_RAD1 &&fabs(leg_root.joint_rec_pos(8))<=100*_RAD1
                && fabs(leg_root.joint_rec_pos(9))<=90*_RAD1 && fabs(leg_root.joint_rec_pos(10))<=90*_RAD1 &&fabs(leg_root.joint_rec_pos(11))<=100*_RAD1
                && fabs(leg_root.joint_rec_pos(12))<=90*_RAD1 && fabs(leg_root.joint_rec_pos(13))<=90*_RAD1 &&fabs(leg_root.joint_rec_pos(14))<=100*_RAD1
                && fabs(leg_root.joint_rec_pos(15))<=90*_RAD1 && fabs(leg_root.joint_rec_pos(16))<=90*_RAD1 &&fabs(leg_root.joint_rec_pos(17))<=100*_RAD1
                )
                {
                        for(int i=0;i<6;i++)
                        {
                                leg_root.joint_des_pos.block<3,1>(0,i) =leg_root.joint_rec_pos.block<1,3>(0,i*3);
                        }
                        printf("\n------------------------------!!!!!!---------------------------------------\n");
                        std::cout<<"leg_root.joint_des_pos"<<std::endl;
                        std::cout<<leg_root.joint_des_pos*_RAD2<<std::endl;
                        get_joint_pos_flag=true;    //lcc 20230405:    如果接收到了电机的当前位置点书籍，那么motor_pos_get_flag=true;                      
                        mtx2_flag=1;
                }
                else
                {
                        printf("\n ----------------knee angle >=100------------- \n");
                }
                // mtx2.unlock();
        }  
        if(joint_pos_run_count>=3) // lcc 20230405: 表示已经从机器人起始点角度向目标角度以等差逼近的方式，逼近了两步
                {
                        _dataUnuProtect[0].velLimAndDifFroDesPosAndActPos(0, 3,
                                                                        leg_root.joint_des_pos.block<3, 1>(0, 0), 
                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 0).transpose(), 20 * _RAD1,
                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 0).transpose(), 9);
                        _dataUnuProtect[1].velLimAndDifFroDesPosAndActPos(1,3,
                                                                        leg_root.joint_des_pos.block<3, 1>(0, 1), 
                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 3).transpose(), 20 * _RAD1,
                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 3).transpose(), 9);
                        _dataUnuProtect[2].velLimAndDifFroDesPosAndActPos(2, 3,
                                                                        leg_root.joint_des_pos.block<3, 1>(0, 2), 
                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 6).transpose(), 20 * _RAD1,
                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 6).transpose(), 9);
                        _dataUnuProtect[3].velLimAndDifFroDesPosAndActPos(3, 3,
                                                                        leg_root.joint_des_pos.block<3, 1>(0, 3), 
                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 9).transpose(), 20 * _RAD1,
                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 9).transpose(), 9);
                        _dataUnuProtect[4].velLimAndDifFroDesPosAndActPos(4, 3,
                                                                        leg_root.joint_des_pos.block<3, 1>(0, 4), 
                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 12).transpose(), 20 * _RAD1,
                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 12).transpose(), 9);
                        _dataUnuProtect[5].velLimAndDifFroDesPosAndActPos(5, 3,
                                                                        leg_root.joint_des_pos.block<3, 1>(0, 5), 
                                                                        leg_root.joint_rec_pos.block<1, 3>(0, 15).transpose(), 20 * _RAD1,
                                                                        leg_root.joint_rec_vel.block<1, 3>(0, 15).transpose(), 9);
                        // 如果有false,那么realHexMsgLoadAndSend()就不会执行
                        if (_dataUnuProtect[5].diff_val_flag == false or _dataUnuProtect[4].diff_val_flag == false 
                        or _dataUnuProtect[3].diff_val_flag == false or _dataUnuProtect[2].diff_val_flag == false 
                        or _dataUnuProtect[1].diff_val_flag == false or _dataUnuProtect[0].diff_val_flag == false)
                        {
                                _dataUnuProtect[5].diff_val_flag = false;
                                _dataUnuProtect[4].diff_val_flag = false;
                                _dataUnuProtect[3].diff_val_flag = false;
                                _dataUnuProtect[2].diff_val_flag = false;
                                _dataUnuProtect[1].diff_val_flag = false;
                                _dataUnuProtect[0].diff_val_flag = false;
                                // printf("\n   ------------diff_val_flag:%d --------------\n",_dataUnuProtect[5].diff_val_flag);
                        }
                        else if(_dataUnuProtect[5].vel_lim_flag==false or _dataUnuProtect[4].vel_lim_flag==false 
                        or _dataUnuProtect[3].vel_lim_flag==false or _dataUnuProtect[2].vel_lim_flag==false 
                        or _dataUnuProtect[1].vel_lim_flag==false or _dataUnuProtect[0].vel_lim_flag==false)
                        {
                                _dataUnuProtect[5].vel_lim_flag=false;_dataUnuProtect[4].vel_lim_flag=false;
                                _dataUnuProtect[3].vel_lim_flag=false;_dataUnuProtect[2].vel_lim_flag=false;
                                _dataUnuProtect[1].vel_lim_flag=false;_dataUnuProtect[0].vel_lim_flag=false;
                                // printf("\n   ------------vel_lim_flag:%d --------------\n",_dataUnuProtect[5].vel_lim_flag);
                        }
                        else
                        {
                                realHexMsgLoadAndSend();
                        }
                }
}

/**
* @details 1.设置电机在参数：pos,vel,tor,kp,kd; 
* @details 2.装载数据; 
* @details 3.设置期望角度与真实角度在容忍值，发送; 
*/
int runaaa=0;
std::mutex mtx;
void Hexapod::realHexMsgLoadAndSend(void)
{

    #if HARD_WARE==1

        mtx.lock();

                HarBri.lf_des_pos=leg_root.joint_des_pos.block<3, 1>(0, 0);
                HarBri.lm_des_pos=leg_root.joint_des_pos.block<3, 1>(0, 1);
                HarBri.lb_des_pos=leg_root.joint_des_pos.block<3, 1>(0, 2);
                HarBri.rf_des_pos=leg_root.joint_des_pos.block<3, 1>(0, 3);
                HarBri.rm_des_pos=leg_root.joint_des_pos.block<3, 1>(0, 4);
                HarBri.rb_des_pos=leg_root.joint_des_pos.block<3, 1>(0, 5);

                HarBri.lf_des_tor.setZero();
                HarBri.lm_des_tor.setZero();
                HarBri.lb_des_tor.setZero();
                HarBri.rf_des_tor.setZero();
                HarBri.rm_des_tor.setZero();
                HarBri.rb_des_tor.setZero();

                HarBri.lf_des_vel.setZero();
                HarBri.lm_des_vel.setZero();
                HarBri.lb_des_vel.setZero();
                HarBri.rf_des_vel.setZero();
                HarBri.rm_des_vel.setZero();
                HarBri.rb_des_vel.setZero();

                Eigen::Vector3d kp_temp, kd_temp;
                kp_temp<< 320, 320, 320;
                kd_temp<< 3, 3, 3;
                // kp_temp<< 20, 20, 20;
                // kd_temp<< 2, 2, 2;
                // kp_temp<< 80, 80, 80;
                // kd_temp<< 2, 2, 2;
                HarBri.lf_des_kp=kp_temp;
                HarBri.lm_des_kp=kp_temp;
                HarBri.lb_des_kp=kp_temp;
                HarBri.rf_des_kp=kp_temp;
                HarBri.rm_des_kp=kp_temp;
                HarBri.rb_des_kp=kp_temp;

                HarBri.lf_des_kd=kd_temp;
                HarBri.lm_des_kd=kd_temp;
                HarBri.lb_des_kd=kd_temp;
                HarBri.rf_des_kd=kd_temp;
                HarBri.rm_des_kd=kd_temp;
                HarBri.rb_des_kd=kd_temp;

                HarBri.lfDataLoading(HarBri.lf_des_pos, HarBri.lf_des_vel, HarBri.lf_des_tor, HarBri.lf_des_kp, HarBri.lf_des_kd);
                HarBri.lmDataLoading(HarBri.lm_des_pos, HarBri.lm_des_vel, HarBri.lm_des_tor, HarBri.lm_des_kp, HarBri.lm_des_kd);
                HarBri.lbDataLoading(HarBri.lb_des_pos, HarBri.lb_des_vel, HarBri.lb_des_tor, HarBri.lb_des_kp, HarBri.lb_des_kd);
                HarBri.rfDataLoading(HarBri.rf_des_pos, HarBri.rf_des_vel, HarBri.rf_des_tor, HarBri.rf_des_kp, HarBri.rf_des_kd);
                HarBri.rmDataLoading(HarBri.rm_des_pos, HarBri.rm_des_vel, HarBri.rm_des_tor, HarBri.rm_des_kp, HarBri.rm_des_kd);
                HarBri.rbDataLoading(HarBri.rb_des_pos, HarBri.rb_des_vel, HarBri.rb_des_tor, HarBri.rb_des_kp, HarBri.rb_des_kd);

                // HarBri.showAllData();
                // runaaa++;
                // printf("----------------realHexMsgLoadAndSend run:%d-------------------\n",runaaa);
        mtx.unlock();

    #endif
}

