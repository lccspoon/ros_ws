#include"QPForce712.h"
using namespace std;

void QPForce712::initialize() {
  Dt = 0.002;//计算时间

  value_lb.resize(3, 1);  value_ub.resize(3, 1);

  value_lb << -20,-20,0;  value_ub << 20,20,166;//摩擦锥力约束,地面支撑力
  mass = 16.5730;//机身质量
  gravity << 0,0,9.8;//重力加速度
  Kp << 15,5,10,  15,15,2;// xyz rpy 位置误差系数
  Kd << 5,4,5,   5,5,0;// xyz rpy 速度误差系数

  Kpcom.diagonal() = Kp.block(0,0,3,1);  Kdcom.diagonal() = Kd.block(0,0,3,1);
  Kpbase.diagonal() = Kp.block(3,0,3,1);  Kdbase.diagonal() = Kd.block(3,0,3,1);
  Qdiag.resize(6,6);  Qdiag.setIdentity();

  Qdiag << 5,0,0,0,0,0,
           0,1,0,0,0,0,
           0,0,1,0,0,0,
           0,0,0,10,0,0,
           0,0,0,0,10,0,
           0,0,0,0,0,5;//优化权重 xyz rpy 加速度

  Ig.resize(3,3);
  Ig << 0.017383, 0, 0,
        0, 0.025469, 0,
        0, 0, 0.038417;
  Ig = 2.565 * Ig;//转动惯量矩阵,在SW里面可以测到,或者urdf

  contact_state.resize(6, 1);  contact_state.setOnes();
  x_COM_world.resize(3, 1);  xdot_COM_world.resize(3, 1); //实际位置、实际速度
  theta_b_world.resize(3, 1);  omega_b_world.resize(3, 1);  //实际角度 实际角加速度
  x_COM_world_desired.resize(3, 1);  xdot_COM_world_desired.resize(3, 1);  //期望位置 期望速度
  theta_b_world_desired.resize(3,1);  omega_b_world_desired.resize(3, 1);  //期望角度 期望角加速度
  x_COM_world_desired.setZero();  xdot_COM_world_desired.setZero();
  theta_b_world_desired.setZero();  omega_b_world_desired.setZero();
  QPforce.setZero();  FeedFf.setZero();

  H_eigen.resize(18, 18);  Ffr.resize(18, 18);
  g_eigen.resize(18, 1);//优化的矩阵
  xOpt_eigen.resize(18, 1);  xOpt_eigen.setZero();
  ub.resize(18, 1);  lb.resize(18, 1);

  mu_friction = 0.6;//摩擦系数,一般不改
  uf = pow(2, 0.5) * mu_friction / 2;
  Friction_constriant << 1, 0, -uf,
                        0, 1, -uf,
                        0, 0, -1;
  BB.setZero();
  CoMVirtualForce.setZero();
}
void QPForce712::Setfeedforward(Eigen::Matrix<double,18,1> _feedf){
  FeedFf = _feedf;//用于设置MPC力矩,用不到
}
void QPForce712::SetQWeights(Eigen::Matrix<double,6,6> Qset){
  Qdiag = Qset;
}//没用到
void QPForce712::set_desiredTrajectoryData(Eigen::Matrix<double,12,1>x_des_in) {
  x_COM_world_desired = x_des_in.block(0,0,3,1);
  xdot_COM_world_desired = x_des_in.block(3,0,3,1);
  theta_b_world_desired = x_des_in.block(6,0,3,1);
  omega_b_world_desired = x_des_in.block(9,0,3,1);
}
void QPForce712::SetContactData(Eigen::Matrix<int,6,1> contact_state_in) {
  contact_state = contact_state_in;
  contact_legnum = contact_state.sum();
}
//实际质心位置 速度 角度 角加速度 ，足端相对于机身坐标系的位置
void QPForce712::updateProblemData(Eigen::Matrix<double,12,1> xfb_in,Eigen::Matrix<double,18,1> p_feet_in) {
  x_COM_world = xfb_in.block(0,0,3,1);  xdot_COM_world = xfb_in.block(3,0,3,1);
  theta_b_world = xfb_in.block(6,0,3,1);  omega_b_world = xfb_in.block(9,0,3,1);
  r_feet << p_feet_in.block(0,0,3,1),p_feet_in.block(3,0,3,1),
            p_feet_in.block(6,0,3,1),p_feet_in.block(9,0,3,1),
            p_feet_in.block(12,0,3,1),p_feet_in.block(15,0,3,1);
  if (contact_legnum) compute_mpc();
}
//r_feet 是3×6的矩阵 TA 是6×18的矩阵
void QPForce712::compute_mpc(){
  for (int i = 0; i < 6; i++){
    Ffr.block(i * 3, i * 3, 3, 3) = contact_state(i)*Friction_constriant;
    ub.block(i * 3, 0, 3, 1) = contact_state(i)*value_ub;
    lb.block(i * 3, 0, 3, 1) = contact_state(i)*value_lb;
    TA.block(0,i*3,3,3) = contact_state(i)*Eigen::MatrixXd::Identity(3,3);  
    TA.block(3,i*3,3,3) = contact_state(i)*cross_mat(r_feet.block(0,i,3,1));
  }

  go.block(0,0,3,1) = mass*(
      Kpcom*(x_COM_world_desired - x_COM_world) + Kdcom*(xdot_COM_world_desired-xdot_COM_world)
      + gravity
      );

  go.block(3,0,3,1) = Ig*(Kpbase*(theta_b_world_desired-theta_b_world) 
      + Kdbase*(omega_b_world_desired-omega_b_world));

  // go -= TA*FeedFf;//没有MPC
  // CoMVirtualForce = go;

  H_eigen = 2 * (TA.transpose() * Qdiag * TA);
  g_eigen = (-2) * TA.transpose() * Qdiag * go;
  // xOpt_eigen = qpOASES_solver(H_eigen,g_eigen,Ffr,BB,lb,ub);
  xOpt_eigen = QPsolver(H_eigen,g_eigen,Ffr,BB,lb,ub);
  QPforce = xOpt_eigen;
}
void QPForce712::copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd& source, int nRows, int nCols) {
  int count = 0;
  for (int i = 0; i < nRows; i++) {
    for (int j = 0; j < nCols; j++) {
      target[count] = source(i, j);
      count++;
    }
  }
}
//通过xyz方向的扭矩反推导矩阵A
Eigen::Matrix3d QPForce712::cross_mat(Eigen::Vector3d r){
  Eigen::Matrix3d cm;
  cm << 0.f, -r(2), r(1),
    r(2), 0.f, -r(0),
    -r(1), r(0), 0.f;
  return cm;
}

Eigen::MatrixXd QPForce712::QPsolver(Eigen::MatrixXd H1,Eigen::MatrixXd f,Eigen::MatrixXd Ffr,Eigen::MatrixXd BB,Eigen::MatrixXd lb,Eigen::MatrixXd ub){
  int NumberOfVariables = H1.rows();
  int NumberOfConstraints = 2*NumberOfVariables;
  Eigen::SparseMatrix<double> hessian;
  Eigen::VectorXd gradient;
  Eigen::SparseMatrix<double> linearMatrix;
  Eigen::MatrixXd linearMatrix1;
  Eigen::VectorXd lowerBound;
  Eigen::VectorXd lowerBound_infty;
  Eigen::VectorXd upperBound;
  hessian.resize(NumberOfVariables, NumberOfVariables);
  for (int i = 0; i < NumberOfVariables; i++) {
    for (int j = 0; j < NumberOfVariables; j++) {
      if (H1(i,j)!=0) {
        hessian.insert(i,j) = H1(i,j);
      }
    }
  }
  gradient.resize(NumberOfVariables);
  gradient = f;
  linearMatrix.resize(NumberOfConstraints, NumberOfVariables);
  linearMatrix1 = Eigen::MatrixXd::Zero(NumberOfConstraints, NumberOfVariables);
  linearMatrix1 << Ffr,Eigen::MatrixXd::Identity(NumberOfVariables, NumberOfVariables);
  for (int i = 0; i < NumberOfConstraints; i++) {
    for (int j = 0; j < NumberOfVariables; j++) {
      if (linearMatrix1(i,j)!=0) {
        linearMatrix.insert(i,j) = linearMatrix1(i,j);
      }
    }
  }
  lowerBound.resize(NumberOfConstraints);
  lowerBound_infty.resize(NumberOfVariables);
	lowerBound_infty.setOnes();
	for (int i = 0; i < NumberOfVariables; i++) {
		lowerBound_infty(i) = -OsqpEigen::INFTY;
	}
  lowerBound << lowerBound_infty, lb;
  upperBound.resize(NumberOfConstraints);
  upperBound << BB, ub;
  OsqpEigen::Solver solver;
  // settings
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  // set the initial data of the QP solver
  solver.data()->setNumberOfVariables(NumberOfVariables);
  solver.data()->setNumberOfConstraints(NumberOfConstraints);
  solver.data()->setHessianMatrix(hessian);
  solver.data()->setGradient(gradient);
  solver.data()->setLinearConstraintsMatrix(linearMatrix);
  solver.data()->setLowerBound(lowerBound);
  solver.data()->setUpperBound(upperBound);
  solver.initSolver();
  solver.solve();
  Eigen::VectorXd QPForce;
  QPForce = solver.getSolution();
  return QPForce;
}
Eigen::MatrixXd QPForce712::qpOASES_solver(Eigen::MatrixXd H_eigen,Eigen::MatrixXd g_eigen,
              Eigen::MatrixXd Ffr,Eigen::MatrixXd i_BB,Eigen::MatrixXd lb_eigen,Eigen::MatrixXd ub_eigen){
  int NUM_VARIABLES_QP,NUM_CONSTRAINTS_QP;
  NUM_VARIABLES_QP=H_eigen.rows();
  NUM_CONSTRAINTS_QP=Ffr.rows();
  real_t H_qpOASES[NUM_VARIABLES_QP * NUM_VARIABLES_QP];
  real_t A_qpOASES[NUM_CONSTRAINTS_QP * NUM_VARIABLES_QP];
  real_t g_qpOASES[NUM_VARIABLES_QP];
  real_t lb_qpOASES[NUM_VARIABLES_QP];
  real_t ub_qpOASES[NUM_VARIABLES_QP];
  real_t ubA_qpOASES[NUM_CONSTRAINTS_QP];
  real_t xOpt_qpOASES[NUM_VARIABLES_QP];
  copy_Eigen_to_real_t(H_qpOASES, H_eigen, NUM_VARIABLES_QP, NUM_VARIABLES_QP);
  copy_Eigen_to_real_t(g_qpOASES, g_eigen, NUM_VARIABLES_QP, 1);
  copy_Eigen_to_real_t(A_qpOASES, Ffr, NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
  copy_Eigen_to_real_t(lb_qpOASES, lb_eigen, NUM_CONSTRAINTS_QP, 1);
  copy_Eigen_to_real_t(ub_qpOASES, ub_eigen, NUM_CONSTRAINTS_QP, 1);
  copy_Eigen_to_real_t(ubA_qpOASES, i_BB, NUM_CONSTRAINTS_QP, 1);
  Options options;
  options.printLevel = PL_NONE;
	QProblem example(NUM_VARIABLES_QP,NUM_CONSTRAINTS_QP);
  example.setOptions(options);
  example.setPrintLevel(PL_NONE);
	int_t nWSR = 50;
	example.init(H_qpOASES, g_qpOASES, A_qpOASES, lb_qpOASES, ub_qpOASES, nullptr, ubA_qpOASES, nWSR);
	example.getPrimalSolution(xOpt_qpOASES);
  Eigen::VectorXd QPForce;
	QPForce.resize(NUM_VARIABLES_QP);
	for (int i = 0; i < NUM_VARIABLES_QP; i++) {
		QPForce(i) = xOpt_qpOASES[i];
	}
  example.reset();
  return QPForce;
}