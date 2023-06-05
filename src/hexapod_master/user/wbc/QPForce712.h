#ifndef QPFORCE_H
#define QPFORCE_H

#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<Eigen/QR>
#include<OsqpEigen/OsqpEigen.h>//库OsqpEigen
#include<qpOASES.hpp>//库qpOASES
using namespace qpOASES;

class QPForce712
{
private:
    double mass;
    double Dt;
    Eigen::MatrixXd value_ub, value_lb, ub, lb, lbA; // nullptr 
    Eigen::Matrix<double,18,1> BB;
    Eigen::Matrix<double,6,18>TA;
    Eigen::MatrixXd Qdiag;
    double mu_friction, uf;
    Eigen::Matrix3d Friction_constriant;
    Eigen::MatrixXd Ig;

    Eigen::VectorXi contact_state;
    int contact_legnum;

    Eigen::VectorXd x_COM_world, xdot_COM_world, theta_b_world, omega_b_world, quat_b_world;
    Eigen::VectorXd x_COM_world_desired, xdot_COM_world_desired, theta_b_world_desired, omega_b_world_desired;

    Eigen::MatrixXd H_eigen, Ffr, g_eigen, xOpt_eigen;
    Eigen::Matrix<double,3,6>r_feet;

    Eigen::Matrix<double,6,1> go,Kp, Kd;
    Eigen::Matrix3d Kpbase,Kdbase,Kpcom,Kdcom;
    Eigen::Vector3d eRR,gravity;

    Eigen::Matrix<double,18,1> FeedFf;
    // Eigen::Matrix<double,6,1> FeedFf;
	
    void compute_mpc();

    Eigen::Matrix3d cross_mat(Eigen::Vector3d r);
    void crossMatrix(Eigen::MatrixXd& R, const Eigen::VectorXd& omega);
    void copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd& source, int nRows, int nCols);
    void copy_Eigen_to_double(double* target, Eigen::VectorXd& source, int length);
    void copy_Array_to_Eigen(Eigen::VectorXd& target, double* source, int len, int startIndex);
    void copy_Array_to_Eigen(Eigen::MatrixXd& target, double* source, int len, int startIndex);
    Eigen::MatrixXd qpOASES_solver(Eigen::MatrixXd H_eigen, Eigen::MatrixXd g_eigen,
                Eigen::MatrixXd Ffr, Eigen::MatrixXd i_BB, Eigen::MatrixXd lb_eigen,
                Eigen::MatrixXd ub_eigen);
    Eigen::MatrixXd QPsolver(Eigen::MatrixXd H1,Eigen::MatrixXd f,Eigen::MatrixXd Ffr,Eigen::MatrixXd BB,Eigen::MatrixXd lb,Eigen::MatrixXd ub);
public:
  void initialize();
  void Setfeedforward(Eigen::Matrix<double,18,1> _feedf);
  // void Setfeedforward(Eigen::Matrix<double,6,1> _feedf);
  void SetQWeights(Eigen::Matrix<double,6,6> Qset);
  void SetContactData(Eigen::Matrix<int,6,1> contact_state_in);
  void set_desiredTrajectoryData(Eigen::Matrix<double,12,1>x_des_in); 
  void updateProblemData(Eigen::Matrix<double,12,1> xfb_in, 
              Eigen::Matrix<double,18,1> p_feet_in);
  Eigen::Matrix<double,18,1> QPforce;
  Eigen::Matrix<double,6,1> CoMVirtualForce;
  Eigen::Matrix<double,6,1> CoMVirtualForce2;
};

#endif