#ifndef _PID_CONTROLLER_H
#define _PID_CONTROLLER_H

class pid_controller
{
    private:
        double err=0;
        double err_next=0;
        double err_last=0;
        double _kp=0;
        double _kd=0;
        double _ki=0;
        double integral=0;
        double increment_val=0;
        double ret_value;

    public:

        void setKpKiKd(double kp,double ki,double kd)
        {
            _kp=kp; _ki=ki; _kd=kd;
        }

        double positonPidTau(double des_value, double act_value)
        {

                err=act_value-des_value;

                integral+=err;

                ret_value=  _kp*err +
                            _kd*(err-err_last) +
                            _ki*integral;

                err_last=err;

            return ret_value;
        }

        double incrementPidTau(double des_value,double act_value)
        {
                double ret_value;

                err=act_value-des_value;


                increment_val=      _kp*(err-err_next) +
                                    _kd*(err-2*err_next+err_last) +
                                    _ki*err;

                ret_value+=increment_val;

                err_last=err_next;
                err_next=err;
            return ret_value;
        }


};




#endif
