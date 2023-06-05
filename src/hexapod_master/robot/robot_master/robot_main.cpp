#include"robot_main.h"
#if HARD_WARE==2
        ROSLegTopicHandle GazeboSim;
#endif
RobotInterface HexInt;


KeyBoardControl KeyBoardCtrl;


A1BasicEKF StateEstimator;  //lcc 20230428
bool _state_estimator_flag=true;  //lcc 20230428
std::mutex _state_estimator_mut;

