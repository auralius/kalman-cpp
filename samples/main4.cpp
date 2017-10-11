/**
 * @file main4.cpp
 * @author Auralius Manurung
 * @date 18 Apr 2015
 * 
 * @brief Example for the extended Kalman filter.
 * 
 * This example is taken from 
 * <a href="http://ch.mathworks.com/matlabcentral/fileexchange/38302-kalman-filter-package/content//Kalman%20Filter%20Package/Examples/ExtendedKalmanFilterDemo.m">here</a>.
 */

#include <fstream>

#include "ekf.h"

/// @cond DEV
/*
 * Class EKF needs to be derived, two virtual functions are provided in 
 * which system model and output model are described.
 */
class MyEKF: public EKF
{
public:  
  virtual colvec f(const colvec& x, const colvec& u) {
    colvec xk(nOutputs_);
    xk(0) = sin(x(1) * u(0));
    xk(1) = x(1);
    return xk;
  }
  
  virtual colvec h(const colvec& x) {
    colvec zk(nOutputs_);
    zk(0) = x(0);
    zk(1) = x(1);
    return zk;
  }
};
/// @endcond

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  /* 
   * Log the result into a tab delimitted file, later we can open 
   * it with Matlab. Use: plot_data4.m to plot the results.
   */
  ofstream log_file;
  log_file.open("log_file4.txt");
  
  int n_states = 2;
  int n_outputs = 2;
  mat Q(2, 2);
  mat R(2, 2);
  
  Q << 0.001 << 0    << endr
    << 0     << 0    << endr;
    
  R << 0.1   << 0    << endr
    <<   0   << 0.01 << endr;
  
  colvec x0(2);
  x0 << 0 << 1 * M_PI / 500;
  
  colvec u(1);
   
  MyEKF myekf;
  myekf.InitSystem(n_states, n_outputs, Q, R);
  myekf.InitSystemState(x0);
  
  for (int k = 0; k < 1000; k ++) {
    u(0) = k;
    myekf.EKalmanf(u);
    
    colvec *x = myekf.GetCurrentState();
    colvec *x_m = myekf.GetCurrentEstimatedState();
    colvec *z = myekf.GetCurrentOutput();
    
    log_file << k 
             << '\t' << z->at(0,0) << '\t' << x->at(0,0) << '\t' << x_m->at(0,0)
             << '\t' << z->at(1,0) << '\t' << x->at(1,0) << '\t' << x_m->at(1,0)
             << '\n'; 
  }
  
  log_file.close();
  
  return 0;
}