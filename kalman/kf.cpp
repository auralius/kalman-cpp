/**
 * @file kf.cpp
 * @author Auralius Manurung
 * @date 18 Apr 2015
 * @brief Kalman filter implementation, for a linear system.
 */

#include "kf.h"

KF::KF()
{

}

KF::~KF()
{
  
}

void KF::InitSystem(const mat& A, const mat& B, const mat& H, const mat& Q, const mat& R)
{
  A_ = A;
  B_ = B;
  H_ = H;
  Q_ = Q;
  R_ = R;
    
  // Stdev is sqrt of variance
  sqrt_Q_ = sqrt(Q_);
  sqrt_R_ = sqrt(R_);
        
  int n_states = A.n_cols;
  int n_outputs = H.n_rows;
  
  assert(A.is_square() && "Whoops, A must be a square matrix (n_outputs x n_outputs)");
  assert(B.n_rows == A.n_rows && "Whoops, B has wrong dimension");
  assert(Q.is_square() && "Whoops, Q must be a square matrix");
  assert(R.is_square() && "Whoops, R must be a square matrix (n_outputs x n_outputs)");
  
  // Apply intial states
  x_.resize(n_states);
  x_ =  x_.zeros();
  
  x_p_.resize(n_states);
  x_m_.resize(n_states);
  
  P_p_.resize(n_states, n_states);
  P_m_.resize(n_states, n_states);
  
  v_.resize(n_states);
  w_.resize(n_outputs);
  
  // Inital values:
  P_m_ = P_m_.eye();
  x_m_ = x_m_.zeros();
}

void KF::InitSystemState(const colvec& x0)
{
  arma_assert_same_size(x_.n_rows, x_.n_cols, x0.n_rows, x0.n_cols, "Whoops, error initializing system states");
  x_ = x0;
  x_m_ = x0;
}

void KF::InitStateCovariance(const mat& P0)
{
  arma_assert_same_size(P_m_.n_rows, P_m_.n_cols, P0.n_rows, P0.n_cols, "Whoops, error initializing state covariance");
  P_m_ = P0;
}
  
void KF::Kalmanf(const colvec& u)
{
  // Simulate true system, with noise
  // randn uses a normal/Gaussian distribution with zero mean and unit variance
  v_.randn(A_.n_rows);
  w_.randn(H_.n_rows);
  v_ = sqrt_Q_ * v_;
  w_ = sqrt_R_ * w_;
  x_ = A_ * x_ + B_ * u + v_;
  z_ = H_ * x_ + w_;
  
  // Prior update:
  x_p_ = A_ * x_m_ + B_ * u;
  P_p_ = A_ * P_m_ * trans(A_) + Q_;
  
  // Measurement update:
  mat K = P_p_ * trans(H_) * inv(H_ * P_p_ * trans(H_) + R_);
  x_m_ = x_p_ + K * (z_ - H_ * x_p_);
  P_m_ = P_p_ - K * H_ * P_p_;
  
  // Estimated output is the projection of etimated states to the output function
  z_m_ = H_ * x_m_;
}

void KF::Kalmanf(const colvec& z, const colvec& u)
{
  // Prior update:
  x_p_ = A_ * x_m_ + B_ * u;
  P_p_ = A_ * P_m_ * trans(A_) + Q_;
  
  // Measurement update:
  mat K = P_p_ * trans(H_) * inv(H_ * P_p_ * trans(H_) + R_);
  x_m_ = x_p_ + K * (z - H_ * x_p_);
  P_m_ = P_p_ - K * H_ * P_p_;
  
  // Estimated output is the projection of etimated states to the output function
  z_m_ = H_ * x_m_;
}

colvec* KF::GetCurrentState()
{
    return & x_;
}

colvec* KF::GetCurrentOutput()
{
    return &z_;
}

colvec* KF::GetCurrentEstimatedState()
{
    return &x_m_;
}

colvec* KF::GetCurrentEstimatedOutput()
{
    return &z_m_;
}



