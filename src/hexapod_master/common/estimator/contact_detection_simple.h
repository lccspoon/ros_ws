#ifndef CONTACT_DETECTION_SIMPLE
#define CONTACT_DETECTION_SIMPLE

#include<iostream>
#include <Eigen/Core>
#include <iostream>
#include <Eigen/Dense>
#include<math.h>
#include "../../common/utilities/timer.h"
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>


class contact_detection_simple
{
    private:
        Eigen::Matrix<double,3,6> contact_estimate;
    public:
        #define NUM_LEG 6 
        #define X_THRESHOLD 1 
        #define Y_THRESHOLD 1
        #define Z_THRESHOLD 1 


        Eigen::Matrix<double,1,6> contact_estimate_schedual;
        Eigen::Matrix<double,1,6> contact_estimation(Eigen::Matrix<double,3,6> footend_force);


};



#endif
