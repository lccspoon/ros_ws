#ifndef _PID_CONTROLLER_H
#define _PID_CONTROLLER_H

class pid_controller
{
    private:
        double err=0;
        double err__last_last=0;
        double err_last=0;
        double _kp=0;
        double _kd=0;
        double _ki=0;
        double integral=0;
        double increment_val=0;
        double ret_value;

        double pid_delta=0;

    public:

        void setKpKiKd(double kp,double ki,double kd)
        {
            _kp=kp; _ki=ki; _kd=kd;
        }

        double positonPidTau(double des_value, double act_value)
        {

                err=des_value-act_value;

                integral+=err;

                ret_value=  _kp*err +
                            _kd*(err-err_last) +
                            _ki*integral;

                err_last=err;

            return ret_value;
        }

        double incrementPidTau(double des_value,double act_value,double dt)
        {

            err=des_value-act_value;

            pid_delta = _kp*(err-err_last) + _ki*(err*dt) + _kd*(err-2*err_last+err__last_last);

            err__last_last=err_last;
            err_last=err;

            ret_value=ret_value+pid_delta;

            return ret_value;
        }


};




#endif