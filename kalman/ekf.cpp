/**
 * @file ekf.cpp
 * @author Auralius Manurung
 * @date 18 Apr 2015
 * @brief Extended Kalman filter implementation, for a nonlinear system.
 */

#include "ekf.h"

EKF::EKF()
{

}

EKF::~EKF()
{

}

void EKF::InitSystem(int n_states, int n_outputs, const mat& Q, const mat& R)
{
  assert(Q.is_square() && "Whoops, Q must be a square matrix");
  assert(R.is_square() && "Whoops, R must be a square matrix (n_outputs x n_outputs)");
  
  // Epsilon for computing the Jacobian numerically
  epsilon_= 1e-8;
  
  nStates_ = n_states;
  nOutputs_ = n_outputs;
  
  Q_ = Q;
  R_ = R;
  
  // Stdev is sqrt of variance
  sqrt_Q_ = sqrt(Q_);
  sqrt_R_ = sqrt(R_);

  x_.resize(n_states);
  F_.resize(n_states, n_states);
  H_.resize(n_outputs, n_states);
  
  x_p_.resize(n_states);
  x_m_.resize(n_states);
  
  P_p_.resize(n_states, n_states);
  P_m_.resize(n_states, n_states);
  
  v_.resize(n_states);
  w_.resize(n_outputs);
  
  // Apply intial states
  x_.resize(n_states);
  x_ = x_.zeros();
  
  // Inital values for the Kalman iterations
  P_m_ = P_m_.eye();
  x_m_ = x_m_.zeros();
}

colvec EKF::f(const colvec &x, const colvec &u)
{
  colvec xk(nOutputs_);
  xk = xk.zeros();
  return xk;
}

colvec EKF::h(const colvec &x)
{
  colvec zk(nOutputs_);
  zk = zk.zeros();
  return zk;
}

void EKF::CalcF(const colvec &x, const colvec &u)
{
  colvec f0 = f(x, u);
  
  colvec fn;
  
  for (int j = 0; j < nStates_; j ++) {
    colvec x_eps = x;
    x_eps(j) = x_eps(j) + epsilon_;
    fn = f(x_eps, u);
    
    for (int i = 0; i < nStates_; i ++) {
      F_(i, j) = (fn(i) - f0(i)) / epsilon_;     
    }
  }
  
  //F_.print();
}

void EKF::CalcH(const colvec &x)
{
  colvec h0 = h(x);
  
  colvec hn;
  
  for (int j = 0; j < nStates_; j ++) {
    colvec x_eps = x;
    x_eps(j) = x_eps(j) + epsilon_;
    hn = h(x_eps);
   
    for (int i = 0; i < nOutputs_; i ++) 
      H_(i, j) = (hn(i) - h0(i)) / epsilon_;    
  }
  
  //H_.print();
}

void EKF::InitSystemState(const colvec& x0)
{
  arma_assert_same_size(x_.n_rows, x_.n_cols, x0.n_rows, x0.n_cols, "Whoops, error initializing system states");
  x_ = x0;
  x_m_ = x0;
}

void EKF::InitSystemStateCovariance(const mat& P0)
{
  arma_assert_same_size(P0.n_rows, P0.n_cols, P_m_.n_rows, P_m_.n_cols, "Whoops, error initializing state covariance");
  P_m_ = P0;
}


void EKF::EKalmanf(const colvec& u)
{  
  // Simulate true system, with noise
  // randn uses a normal/Gaussian distribution with zero mean and unit variance
  v_.randn(F_.n_rows);
  w_.randn(H_.n_rows);
  v_ = sqrt_Q_ * v_;
  w_ = sqrt_R_ * w_;
  x_ = f(x_, u) + v_;
  z_ = h(x_) + w_;
  
  CalcF(x_m_, u);
  CalcH(x_m_);
  
  // Prior update:
  x_p_ = f(x_m_, u);
  P_p_ = F_ * P_m_ * trans(F_) + Q_;
  
  // Measurement update:
  mat K = P_p_ * trans(H_) * inv(H_ * P_p_ * trans(H_) + R_);
  x_m_ = x_p_ + K * (z_ - h(x_p_));
  P_m_ = P_p_ - K * H_ * P_p_;
  
  // Estimated output is the projection of etimated states to the output function
  z_m_ = h(x_m_);
}

void EKF::EKalmanf(const colvec& z, const colvec& u)
{    
  CalcF(x_m_, u);
  CalcH(x_m_);
  
  // Prior update:
  x_p_ = f(x_m_, u);
  P_p_ = F_ * P_m_ * trans(F_) + Q_;
  
  // Measurement update:
  mat K = P_p_ * trans(H_) * inv(H_ * P_p_ * trans(H_) + R_);
  x_m_ = x_p_ + K * (z - h(x_p_));
  P_m_ = P_p_ - K * H_ * P_p_;

  // Estimated output is the projection of etimated states to the output function
  z_m_ = h(x_m_);
}

colvec* EKF::GetCurrentState()
{
    return & x_;
}

colvec* EKF::GetCurrentOutput()
{
    return &z_;
}

colvec* EKF::GetCurrentEstimatedState()
{
    return &x_m_;
}


colvec* EKF::GetCurrentEstimatedOutput()
{
    return &z_m_;
}



