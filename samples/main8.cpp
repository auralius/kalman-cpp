/**
 * @file main8.cpp
 * @author Auralius Manurung
 * @date 27 Jan 2017
 * 
 * @brief Example for the second order extended Kalman filter.
 *
 * This example is taken from  <a href="http://becs.aalto.fi/en/research/bayes/ekfukf/documentation.pdf">here</a>, 
 * section 3.2: "Tracking a random sine signal".
 */

#include <fstream>

#include "ekf2.h"

/// @cond DEV
/*
 * Class EKF needs to be derived, two virtual functions are provided in 
 * which system model and output model are described.
 */
class MyEKF: public EKF2
{
public:  
  virtual colvec f(const colvec& x, const colvec& u, const int k) {
    colvec xk(nStates_);
    mat A(3, 3);

    A << 1 << dt << 0 << endr
      << 0 << 1  << 0 << endr
      << 0 << 0  << 1 << endr;
      
    xk = A * x;;
    //xk.print("xk=");
    return xk;
  }
  
  virtual colvec h(const colvec& x, const colvec& u, const int k) {
    colvec zk(nOutputs_);
    zk(0) = x(2)*sin(x(0));
    return zk;
  }
  
  static const double dt;
  
};

const double MyEKF::dt = 0.01;

/// @endcond

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  /* 
   * Log the result into a tab delimitted file, later we can open 
   * it with Matlab. Use: plot_data6.m to plot the results.
   */
  ofstream log_file;
  log_file.open("log_file8.txt");
  
  int n_states = 3;
  int n_outputs = 1;
  mat Q(n_states, n_states);
  mat R(n_outputs, n_outputs);
  
   
  MyEKF myekf;
  
  double q1 = 0.2;
  double q2 = 0.1;
  double dt = myekf.dt;
  
  Q << q1*dt*dt*dt/3 << q1*dt*dt/2 << 0     << endr
    << q1*dt*dt/2    << q1*dt      << 0     << endr
    << 0             << 0          << dt*q2 << endr;
    
  R << 1 << endr;
  
  colvec x0(n_states);
  x0 << 0 << 10 << 1;
  
  mat P0 = eye<mat> (3,3);
  P0 = 3 * P0;
  
  colvec u;

  // No inputs
  u = u.zeros(); 
 
  myekf.InitSystem(n_states, n_outputs, Q, R);
  myekf.InitSystemState(x0);
  myekf.InitSystemStateCovariance(P0);
  
  for (int k = 0; k < 500; k ++) {
    myekf.EKalmanf(u, k);
    
    colvec *x = myekf.GetCurrentState();
    colvec *x_m = myekf.GetCurrentEstimatedState();
    colvec *z = myekf.GetCurrentOutput();
    colvec *z_m = myekf.GetCurrentEstimatedOutput();
    
    log_file << k << '\t' << x->at(0,0) << '\t' << x_m->at(0,0)  
                  << '\t' << x->at(1,0) << '\t' << x_m->at(1,0)
                  << '\t' << x->at(2,0) << '\t' << x_m->at(2,0)
                  << '\t' << z->at(0,0) << '\t' << z_m->at(0,0) << '\t' 
                  << '\n'; 
  }
  
  log_file.close();
  
  return 0;
}