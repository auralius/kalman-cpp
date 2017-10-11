/**
 * @file ekf2.cpp
 * @author Auralius Manurung
 * @date 18 Apr 2015
 * @brief Second order extended Kalman filter implementation, for a nonlinear system.
 */

#include "ekf2.h"

EKF2::EKF2()
{

}

EKF2::~EKF2()
{

}

void EKF2::InitSystem(int n_states, int n_outputs, const mat& Q, const mat& R)
{
    assert(Q.is_square() && "Whoops, Q must be a square matrix");
    assert(R.is_square() && "Whoops, R must be a square matrix (n_outputs x n_outputs)");

    // Epsilon for computing the Jacobian numerically
    epsilon_ = 1e-5;

    nStates_ = n_states;
    nOutputs_ = n_outputs;

    Q_ = Q;
    R_ = R;

    // Stdev is sqrt of variance
    sqrt_Q_ = sqrt(Q_);
    sqrt_R_ = sqrt(R_);

    x_.resize(n_states);

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

colvec EKF2::f(const colvec &x, const colvec &u)
{
    colvec xk(nOutputs_);
    xk = xk.zeros();
    return xk;
}

colvec EKF2::h(const colvec &x)
{
    colvec zk(nOutputs_);
    zk = zk.zeros();
    return zk;
}

mat EKF2::CalcFx(const colvec &x, const colvec &u)
{
    mat F;
    F.resize(nStates_, nStates_);
    
    colvec f0 = f(x, u);
    colvec fn;

    for (int j = 0; j < nStates_; j++) {
        colvec x_eps = x;
        x_eps(j) = x_eps(j) + epsilon_;
        fn = f(x_eps, u);

        for (int i = 0; i < nStates_; i++) {
            F(i, j) = (fn(i) - f0(i)) / epsilon_;
        }
    }

    //F_.print();
    return F;
}

mat EKF2::CalcHx(const colvec &x)
{
    mat H;
    H.resize(nOutputs_, nStates_);
    
    colvec h0 = h(x);
    colvec hn;

    for (int j = 0; j < nStates_; j++) {
        colvec x_eps = x; 
        x_eps(j) = x_eps(j) + epsilon_;
        hn = h(x_eps);

        for (int i = 0; i < nOutputs_; i++)
            H(i, j) = (hn(i) - h0(i)) / epsilon_;
    }

    //H_.print();
    return H;
}

mat EKF2::CalcFxx(const colvec &x, const colvec &u, const int i)
{
    mat F2;
    F2.resize(nStates_, nStates_);
    
    for (int j = 0; j < nStates_; j++) {
        
        colvec x0 = x;
        x0(j) = x0(j) + epsilon_;
        mat F_plus = CalcFx(x0, u);
    
        x0 = x;
        x0(j) = x0(j) - epsilon_;
        mat F_min = CalcFx(x0, u);
        
        rowvec dF = (F_plus.row(i) - F_min.row(i)) / (2 * epsilon_);
        F2.row(j) = dF;
    }
    
    //F2.print("F2");
    return F2;
}

mat EKF2::CalcHxx(const colvec &x, const int i)
{
    mat H2;
    H2.resize(nStates_, nStates_);
    
    for (int j = 0; j < nStates_; j++) {
        
        colvec x0 = x;
        x0(j) = x0(j) + epsilon_;
        mat H_plus = CalcHx(x0);
    
        x0 = x;
        x0(j) = x0(j) - epsilon_;
        mat H_min = CalcHx(x0);
       
        rowvec dH = (H_plus.row(i) - H_min.row(i)) / (2 * epsilon_);
        H2.row(j) = dH;
    }
    
    //H2.print("H2");
    return H2;
}

colvec EKF2::e(const int i, const int max_len)
{
    colvec temp(zeros(max_len));
    temp(i) = 1;
    return temp;
}


void EKF2::InitSystemState(const colvec& x0)
{
    arma_assert_same_size(x_.n_rows, x_.n_cols, x0.n_rows, x0.n_cols, "Whoops, error initializing system states");
    x_ = x0;
    x_m_ = x0;
}

void EKF2::InitSystemStateCovariance(const mat& P0)
{
    arma_assert_same_size(P0.n_rows, P0.n_cols, P_m_.n_rows, P_m_.n_cols, "Whoops, error initializing state covariance");
    P_m_ = P0;
}

void EKF2::EKalmanf(const colvec& u)
{
    // Simulate true system, with noise
    // randn uses a normal/Gaussian distribution with zero mean and unit variance
    v_.randn(nStates_);
    w_.randn(nOutputs_);
    v_ = sqrt_Q_ * v_;
    w_ = sqrt_R_ * w_;
    x_ = f(x_, u) + v_;
    z_ = h(x_) + w_;

    mat Fx = CalcFx(x_m_, u);
    mat Hx = CalcHx(x_m_);
    
    // Prior update:
    mat FxxP(zeros(nStates_, 1));
    mat FxxPFxxP(zeros(nStates_, nStates_));
    for (int i = 0; i < nStates_; i ++) {
        mat temp = CalcFxx(x_m_, u, i) * P_m_;
        FxxP = FxxP + e(i, nStates_) * trace(temp);
        FxxPFxxP = FxxPFxxP + e(i, nStates_) * trans(e(i, nStates_)) * trace(temp * temp);
    }
    
    x_p_ = f(x_m_, u) + 0.5 * FxxP;
    P_p_ = Fx * P_m_ * trans(Fx) + 0.5 * FxxPFxxP + Q_;

    // Measurement update:
    colvec HxxP(zeros(nOutputs_));
    colvec HxxPHxxP(zeros(nOutputs_));
    for (int i = 0; i < nOutputs_; i ++) {
        mat temp = CalcHxx(x_p_, i) * P_p_;
        HxxP = HxxP + e(i, nOutputs_) * trace(temp);
        HxxPHxxP = HxxPHxxP + e(i, nOutputs_) * trans(e(i, nOutputs_)) * trace(temp * temp);
    }
    
    mat z_p = h(x_p_) + 0.5 * HxxP;
    mat S = Hx * P_p_ * trans(Hx) + 0.5 * HxxPHxxP + R_;
    mat K = P_p_ * trans(Hx) * inv(S);
    x_m_ = x_p_ + K * (z_ - z_p);
    P_m_ = P_p_ - K * S * trans(K);

    // Estimated output is the projection of etimated states to the output function
    z_m_ = h(x_m_);
}

void EKF2::EKalmanf(const colvec& z, const colvec& u)
{
    mat Fx = CalcFx(x_m_, u);
    mat Hx = CalcHx(x_m_);
    
    // Prior update:
    mat FxxP(zeros(nStates_, 1));
    mat FxxPFxxP(zeros(nStates_, nStates_));
    for (int i = 0; i < nStates_; i ++) {
        mat temp = CalcFxx(x_m_, u, i) * P_m_;
        FxxP = FxxP + e(i, nStates_) * trace(temp);
        FxxPFxxP = FxxPFxxP + e(i, nStates_) * trans(e(i, nStates_)) * trace(temp * temp);
    }
    
    x_p_ = f(x_m_, u) + 0.5 * FxxP;
    P_p_ = Fx * P_m_ * trans(Fx) + 0.5 * FxxPFxxP + Q_;

    // Measurement update:
    colvec HxxP(zeros(nOutputs_));
    colvec HxxPHxxP(zeros(nOutputs_));
    for (int i = 0; i < nOutputs_; i ++) {
        mat temp = CalcHxx(x_p_, i) * P_p_;
        HxxP = HxxP + e(i, nOutputs_) * trace(temp);
        HxxPHxxP = HxxPHxxP + e(i, nOutputs_) * trans(e(i, nOutputs_)) * trace(temp * temp);
    }
    
    mat z_p = h(x_p_) + 0.5 * HxxP;
    mat S = Hx * P_p_ * trans(Hx) + 0.5 * HxxPHxxP + R_;
    mat K = P_p_ * trans(Hx) * inv(S);
    x_m_ = x_p_ + K * (z - z_p);
    P_m_ = P_p_ - K * S * trans(K);

    // Estimated output is the projection of etimated states to the output function
    z_m_ = h(x_m_);
}

colvec* EKF2::GetCurrentState()
{
    return &x_;
}

colvec* EKF2::GetCurrentOutput()
{
    return &z_;
}

colvec* EKF2::GetCurrentEstimatedState()
{
    return &x_m_;
}


colvec* EKF2::GetCurrentEstimatedOutput()
{
    return &z_m_;
}



