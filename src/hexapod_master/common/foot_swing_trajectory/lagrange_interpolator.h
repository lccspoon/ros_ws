#ifndef _LAGRANGEINTERPOLATOR_H
#define _LAGRANGEINTERPOLATOR_H

#include "../utilities/linear_trans.h"
#include <iostream>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cmath>

using namespace Eigen;

class LagrangeInterpolator 
{
    private:
        VectorXd x_;
        VectorXd y_;
        int n_;

    public:
        void setX(const VectorXd& x) 
        {
            x_ = x;
            n_ = x.size();
        }

        void setY(const VectorXd& y) 
        {
            y_ = y;
        }

        VectorXd interpolate(const VectorXd& xi) 
        {
            MatrixXd L = MatrixXd::Ones(xi.size(), n_);

            for (int j = 0; j < n_; j++) {
                for (int i = 0; i < n_; i++) {
                    if (i != j) {
                        L.col(j) = L.col(j).array() * (xi.array() - x_(i)) / (x_(j) - x_(i));
                    }
                }
            }

            VectorXd yi = L * y_;
            return yi;
        }

        void eg(void)
        {
            VectorXd x(9), y(9);
            x << 0, 1, 2, 3, 4, 5, 6, 7, 8;
            y << 0, 20, 30, 40, 35, 25, 16, 10, 0;

            int numPoints = 1000;
            VectorXd xi = VectorXd::LinSpaced(numPoints, x(0), x(x.size() - 1));

            setX(x);
            setY(y);
            VectorXd yi = interpolate(xi);

            // 输出插值点的坐标
            for (int i = 0; i < numPoints; i++) 
            {
                std::cout << "(" << xi(i) << ", " << yi(i) << ")" << std::endl;
            }
        }

        void eg_bytime(double dt)  // dt: 0~1;
        {
            VectorXd x(9), y(9);
            x << 0, 1, 2, 3, 4, 5, 6, 7, 8;
            y << 0, 20, 30, 40, 35, 25, 16, 10, 0;

            int numPoints = 200;
            VectorXd xi = VectorXd::LinSpaced(numPoints, x(0), x(x.size() - 1));

            setX(x);
            setY(y);
            VectorXd yi = interpolate(xi);

            // 输出插值点的坐标
            if(dt<=0) dt=0;
            if(dt>=1) dt=1;
            std::cout << "(" << xi( std::round( (numPoints-1)*dt) ) << ", " << yi( std::round( (numPoints-1)*dt) ) << ")"<< dt << std::endl;
        }

        Vector3d liftLegOne_cm(double _x, double _y, double dt)  // dt: 0~1;
        {
            VectorXd x(2), y(2);
            x << 0, _x*0.01;
            y << 0, _y*0.01;

            int numPoints = 100;
            VectorXd xi = VectorXd::LinSpaced(numPoints, x(0), x(x.size() - 1));

            setX(x);
            setY(y);
            VectorXd yi = interpolate(xi);

            // 输出插值点的坐标
            if(dt<=0) dt=0;
            if(dt>=1) dt=1;
            // std::cout << "(" << xi( std::round( (numPoints-1)*dt) ) << ", " << yi( std::round( (numPoints-1)*dt) ) << ")"<< dt << std::endl;

            Vector3d retPos;
            retPos<< xi( std::round( (numPoints-1)*dt) ), 0, yi( std::round( (numPoints-1)*dt) );

            return retPos;
        }

        int _y_higher=1;
        Vector3d legLiftAccount_cm(double _x, double _y, double dt)  // dt: 0~1;
        {
            VectorXd x(2), y(2);
            x << 0, _x*0.01;

            if(dt==1)
                _y_higher++;

            y << 0, _y_higher * _y*0.01;


            int numPoints = 100;
            VectorXd xi = VectorXd::LinSpaced(numPoints, x(0), x(x.size() - 1));

            setX(x);
            setY(y);
            VectorXd yi = interpolate(xi);

            // 输出插值点的坐标
            if(dt<=0) dt=0;
            if(dt>=1) dt=1;
            // std::cout << "(" << xi( std::round( (numPoints-1)*dt) ) << ", " << yi( std::round( (numPoints-1)*dt) ) << ")"<< dt << std::endl;

            Vector3d retPos;
            retPos<< xi( std::round( (numPoints-1)*dt) ), 0, yi( std::round( (numPoints-1)*dt) );

            return retPos;
        }

};


#endif

