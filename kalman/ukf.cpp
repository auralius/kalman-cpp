/**
 * @file ukf.cpp
 * @author Auralius Manurung
 * @date 13 Peb 2021
 * @brief Unscented Kalman filter implementation, for a nonlinear system.
 */

#include "ukf.h"

UKF::UKF()
{

}

UKF::~UKF()
{

}

void UKF::InitSystem(int n_states, int n_outputs, const mat& Q, const mat& R)
{
  assert(Q.is_square() && 
      "Whoops, Q must be a square matrix");
  assert(R.is_square() && 
      "Whoops, R must be a square matrix (n_outputs x n_outputs)");
  
  nStates_ = n_states;
  nOutputs_ = n_outputs;
  
  Q_ = Q;
  R_ = R;
  
  // Stdev is sqrt of variance
  sqrt_Q_ = sqrt(Q_);
  sqrt_R_ = sqrt(R_);

  v_.resize(n_states);
  w_.resize(n_outputs);

  x_.resize(n_states);
  z_.resize(n_outputs);

  x_m_.resize(n_states);
  z_m_.resize(n_outputs);
  
  P_.resize(n_states, n_states);
  
  // Apply intial states
  x_.resize(n_states);
  x_ = x_.zeros();
  
  // Inital values for the Kalman iterations
  P_ = P_.eye();
  x_m_ = x_m_.zeros();
}

colvec UKF::f(const colvec &x, const colvec &u)
{
  colvec xk(nOutputs_);
  xk = xk.zeros();
  return xk;
}

colvec UKF::h(const colvec &x)
{
  colvec zk(nOutputs_);
  zk = zk.zeros();
  return zk;
}

void UKF::InitSystemState(const colvec& x0)
{
  arma_assert_same_size(x_.n_rows, x_.n_cols, x0.n_rows, x0.n_cols, 
      "Whoops, error initializing system states");
  x_ = x0;
  x_m_ = x0;
}

void UKF::InitSystemStateCovariance(const mat& P0)
{
  arma_assert_same_size(P0.n_rows, P0.n_cols, P_.n_rows, P_.n_cols, 
      "Whoops, error initializing state covariance");
  P_ = P0;
}


void UKF::UKalmanf(const colvec& u)
{
    // Simulate true system, with noise
    // randn uses a normal/Gaussian distribution, zero mean and unit variance
    v_.randn(nStates_);
    w_.randn(nOutputs_);
    v_ = sqrt_Q_ * v_;
    w_ = sqrt_R_ * w_;
    x_ = f(x_, u) + v_;
    z_ = h(x_) + w_;

 
    // Calculate sigma points
    double alpha = 1e-3;                            // default, tunable
    double ki = 0;                                  // default, tunable
    double beta = 2;                                // default, tunable
    double lambda = alpha * alpha * (nStates_ + ki) - nStates_;  
    double c = nStates_ + lambda;                          

    mat Wm = ones(1, 2 * nStates_ + 1);
    Wm = Wm * 0.5 / c;
    Wm.at(0, 0) = lambda / c;                                // weights for means
    mat Wc = Wm;
    Wc.at(0, 0) = Wc.at(0, 0) + (1 - alpha * alpha + beta);  // weights for covariance

    mat A = sqrt(c) * chol(P_, "lower");

    mat B = repmat(x_m_, 1, nStates_);
    mat X = x_m_;
    X.insert_cols(1, B + A);;
    X.insert_cols(1 + nStates_, B - A);;

    // Unscented transformation of process
    mat X_post;
    colvec x_pre(nStates_);
    x_pre.zeros(nStates_);

    for (uword k = 0; k < X.n_cols; k++)
    {
        X_post.insert_cols(k, f(X.col(k), u));
        x_pre = x_pre + Wm.at(0,k)*X_post.col(k);
    }
    mat dX = X_post - repmat(x_pre, 1, X.n_cols);   
    mat Pxx = dX * diagmat(Wc) * trans(dX) + Q_;

    // Unscented transformation of measurement
    mat Y_post;
    colvec y_pre;
    y_pre.zeros(nOutputs_);

    for (uword k = 0; k < X_post.n_cols; k++)
    {
        Y_post.insert_cols(k, h(X_post.col(k)));
        y_pre = y_pre + Wm.at(0, k)*Y_post.col(k);
    }
    mat dY = Y_post - repmat(y_pre, 1, X_post.n_cols);
    mat Pyy = dY * diagmat(Wc) * trans(dY) + R_;
    
    // Covariance update
    mat Pxy = dX * diagmat(Wc) * trans(dY);        // transformed cross-covariance
    mat K = Pxy * inv(Pyy);
    x_m_ = x_pre + K * (z_ - y_pre);               // state update
    P_ = Pxx - K * Pyy * trans(K);  

    // Estimated output
    z_m_ = h(x_m_);  
}

void UKF::UKalmanf(const colvec& z, const colvec& u)
{    
    // Calculate sigma points
    double alpha = 1e-3;                            // default, tunable
    double ki = 0;                                  // default, tunable
    double beta = 2;                                // default, tunable
    double lambda = alpha * alpha * (nStates_ + ki) - nStates_;
    double c = nStates_ + lambda;

    mat Wm = ones(1, 2 * nStates_ + 1);
    Wm = Wm * 0.5 / c;
    Wm.at(0, 0) = lambda / c;                                // weights for means
    mat Wc = Wm;
    Wc.at(0, 0) = Wc.at(0, 0) + (1 - alpha * alpha + beta);  // weights for covariance

    mat A = sqrt(c) * chol(P_, "lower");

    mat B = repmat(x_m_, 1, nStates_);
    mat X = x_m_;
    X.insert_cols(1, B + A);;
    X.insert_cols(1 + nStates_, B - A);;

    // Unscented transformation of process
    mat X_post;
    colvec x_pre(nStates_);
    x_pre.zeros(nStates_);

    for (uword k = 0; k < X.n_cols; k++)
    {
        X_post.insert_cols(k, f(X.col(k), u));
        x_pre = x_pre + Wm.at(0, k)*X_post.col(k);
    }
    mat dX = X_post - repmat(x_pre, 1, X.n_cols);
    mat Pxx = dX * diagmat(Wc) * trans(dX) + Q_;

    // Unscented transformation of measurement
    mat Y_post;
    colvec y_pre;
    y_pre.zeros(nOutputs_);

    for (uword k = 0; k < X_post.n_cols; k++)
    {
        Y_post.insert_cols(k, h(X_post.col(k)));
        y_pre = y_pre + Wm.at(0, k)*Y_post.col(k);
    }
    mat dY = Y_post - repmat(y_pre, 1, X_post.n_cols);
    mat Pyy = dY * diagmat(Wc) * trans(dY) + R_;

    // Covariance update
    mat Pxy = dX * diagmat(Wc) * trans(dY);        // transformed cross-covariance
    mat K = Pxy * inv(Pyy);
    x_m_ = x_pre + K * (z - y_pre);               // state update
    P_ = Pxx - K * Pyy * trans(K);

    // Estimated output
    z_m_ = h(x_m_);
}

colvec* UKF::GetCurrentState()
{
    return & x_;
}

colvec* UKF::GetCurrentOutput()
{
    return &z_;
}

colvec* UKF::GetCurrentEstimatedState()
{
    return &x_m_;
}


colvec* UKF::GetCurrentEstimatedOutput()
{
    return &z_m_;
}


