#ifndef _TIMER_H
#define _TIMER_H

#include <sys/time.h>
#include <time.h>
#include <stdio.h>

class Clock
{
    private:
        int _year,_month,_day,_hour,_min,_sec,_ms,_us;
        int _record_ms;
        struct timeval tTimeVal;
    public:

        Clock(/* args */)
        {
            ;
        }
        ~Clock()
        {
            ;
        }

        void printCurrentTime(void)
        {
            gettimeofday(&tTimeVal, NULL);
            struct tm *tTM = localtime(&tTimeVal.tv_sec);

            _year=tTM->tm_year + 1900;
            _month=tTM->tm_mon + 1;
            _day=tTM->tm_mday;
            _hour=tTM->tm_hour;
            _min=tTM->tm_min;
            _sec=tTM->tm_sec;
            _ms=tTimeVal.tv_usec / 1000;
            _us=tTimeVal.tv_usec % 1000;

            printf("%04d-%02d-%02d %02d:%02d:%02d.%03d.%03d \n", 
            _year,_month,_day,_hour,_min,_sec,_ms,_us);

        }

        void milliSecondTimerStart(void)
        {
            // struct tm *tTM = localtime(&tTimeVal.tv_sec);
            // _record_ms=tTimeVal.tv_usec;
        }

        int milliSecondTimerEndAndRetPeriod(void)
        {
            // struct tm *tTM = localtime(&tTimeVal.tv_sec);
            // _record_ms=tTimeVal.tv_usec-_record_ms;
            return _record_ms;
        }

};


#include <iostream>
#include <chrono>
 
using namespace std;
using namespace std::chrono;
 
class TimeMteter
{
    private:
        time_point<high_resolution_clock>_start;

    public:
    TimeMteter()
    {
        update();
    }
    
    ~TimeMteter()
    {
        ;
    }
    
    void update()
    {
        _start = high_resolution_clock::now();
    }

    //获取秒
    double getTimerSecond()
    {
        return getTimerMicroSec() * 0.000001;
    }

    //获取毫秒
    double getTimerMilliSec()
    {
        return getTimerMicroSec()*0.001;
    }

    //获取微妙
    long long getTimerMicroSec()
    {
        //当前时钟减去开始时钟的count
        return duration_cast<microseconds>(high_resolution_clock::now() - _start).count();
    }

};
 



#endif