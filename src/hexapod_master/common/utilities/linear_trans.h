#ifndef LINEARCONVERTER_H
#define LINEARCONVERTER_H

#include <stdio.h>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
// #include "../foot_swing_trajectory/foo_and_bod_adj_and_map.h"

class linear_trans
{
    private:
        double _RAD1=180/3.14159;
        int _add_count=0;
        double _data_last,_set_last;
        int _add_count_arr[3]={0};
        double _last_arr[3]={0},_set_arr_last[3]={0};
        int cycle_arr[3]={0};
        bool _conver_done_flag=true;
        double data[3], set[3];
    public:

        double linearConvert(double data,double set,int cycle)
        {   
            if(_set_last!=set)
            {
                _add_count=0;
                _set_last=set;
            }

            if(data!=set)
            {   
                if(_add_count==0)
                    _data_last=data;

                data=_data_last+(set-_data_last)*_add_count/cycle;
                _add_count++;
                _conver_done_flag=false;
                // printf("%d _data_last:%f data:%f set:%f \n",_add_count,_data_last,data,set);
                if(_add_count==cycle)
                {
                    data=_data_last=set;
                    _add_count=0;
                    _conver_done_flag=true;
                    // printf("%d _data_last:%f data:%f set:%f \n",_add_count,_data_last,data,set);
                    // printf("Done!\n");
                }
            }
            return data;
        }

        bool retConvDoneFlag(void)
        {
            return _conver_done_flag;
        }

        double * linearConvert(double * data,double * set,int cycle)
        {   
            for (int i = 0; i < 3; i++)
            {

                if(_set_arr_last[i]!=set[i])
                {
                    _add_count_arr[i]=0;
                    _set_arr_last[i]=set[i];
                }

                if(data[i]!=set[i])
                {
                    if(_add_count_arr[i]==0)
                        _last_arr[i]=data[i];

                    data[i]=_last_arr[i]+(set[i]-_last_arr[i])*_add_count_arr[i]/cycle;
                    _add_count_arr[i]++;
                    _conver_done_flag=false;
                    if(_add_count_arr[i]==cycle)
                    {
                        data[i]=_last_arr[i]=set[i];
                        _add_count_arr[i]=0;
                         _conver_done_flag=true;
                        // printf("%d _data_last:%f data:%f set:%f \n",_add_count_arr[i],_last_arr[i],data[i],set[i]);
                    }
                }
            }
            return data;
        }

        Eigen::Vector3d linearConvert(Eigen::Vector3d data_, Eigen::Vector3d set_,int cycle)
        {   
            for (int i = 0; i < 3; i++)
            {

                if(_set_arr_last[i]!=set_(i))
                {
                    _add_count_arr[i]=0;
                    _set_arr_last[i]=set_(i);
                }

                if(data_(i)!=set_(i))
                {
                    if(_add_count_arr[i]==0)
                        _last_arr[i]=data_(i);

                    data_(i)=_last_arr[i]+(set_(i)-_last_arr[i])*_add_count_arr[i]/cycle;
                    _add_count_arr[i]++;
                    _conver_done_flag=false;
                    if(_add_count_arr[i]==cycle)
                    {
                        data_(i)=_last_arr[i]=set_(i);
                        _add_count_arr[i]=0;
                         _conver_done_flag=true;
                        // printf("%d _data_last:%f data:%f set:%f \n",_add_count_arr[i],_last_arr[i],data_(i),set_(i));
                    }
                }
            }

            return data_;
        }

        /**
        * @brief 
        * @author lcc
        */
        double * linearConvertByDifferValue(double * data,double * set,int set_differ_v)
        {   
            for (int i = 0; i < 3; i++)
            {

                if(_set_arr_last[i]!=set[i])
                {
                    _add_count_arr[i]=0;
                    _set_arr_last[i]=set[i];

                    cycle_arr[i]=fabs( data[i]*_RAD1 -  set[i]*_RAD1  ) / set_differ_v;
                    if( cycle_arr[i]<=10)
                         cycle_arr[i]=10;
                }

                printf(" cycle_arr[%d]:%d \n",i,cycle_arr[i]);
                if(data[i]!=set[i])
                {



                    if(_add_count_arr[i]==0)
                        _last_arr[i]=data[i];

                    data[i]=_last_arr[i]+(set[i]-_last_arr[i])*_add_count_arr[i]/cycle_arr[i];
                    _add_count_arr[i]++;
                    _conver_done_flag=false;
                    if(_add_count_arr[i]==cycle_arr[i])
                    {
                        data[i]=_last_arr[i]=set[i];
                        _add_count_arr[i]=0;
                         _conver_done_flag=true;
                        // printf("%d _data_last:%f data:%f set:%f \n",_add_count_arr[i],_last_arr[i],data[i],set[i]);
                    }
                }
            }
            return data;
        }

};




#endif