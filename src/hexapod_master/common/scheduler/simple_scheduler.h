#ifndef _SIMPLE_SCHEDULER_H
#define _SIMPLE_SCHEDULER_H

#include "../utilities/linear_trans.h"
#include <iostream>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>


class simple_scheduler
{
    private:
        double ret_t=0;
        linear_trans a;
        int flag1=0;
        int flag_stop=0;
    public:
        double retSimpleScheduler(double T=1, double dt=0.01)   //返回 0~T
        {
            if(flag_stop==0)
            {
                ret_t=ret_t+dt;

                if(ret_t>=T && flag1==0)
                {
                    ret_t=T;
                    flag1=1;
                }
                else if(flag1==1)
                {
                    ret_t=0;
                    flag1=0;
                }  
            }
            return ret_t;
        }

        double  retTime(void)
        {
            return ret_t;
        }

        void  stopTime(void)
        {
            flag_stop=1;
        }

        void reSet(void)
        {
            ret_t=0;
            flag_stop=0;
        }
        // simple_scheduler(/* args */);
        // ~simple_scheduler();

};

// simple_scheduler::simple_scheduler(/* args */)
// {
// }

// simple_scheduler::~simple_scheduler()
// {
// }




#endif