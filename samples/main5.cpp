/**
 * @file main5.cpp
 * @author Auralius Manurung
 * @date 12 Aug 2015
 * 
 * @brief Example for the extended Kalman filter.
 * 
 * @section DESCRIPTION
 * This example is taken from 
 * <a href="http://ch.mathworks.com/matlabcentral/fileexchange/18189-learning-the-extended-kalman-filter">here</a>.
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
    colvec xk(nStates_);
    xk << x(1) << x(2) << 0.05*x(0)*(x(1)+x(2));
    return xk;
  }
  
  virtual colvec h(const colvec& x) {
    colvec zk(nOutputs_);
    zk(0) = x(0);
    return zk;
  }
};
/// @endcond

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  /* 
   * Log the result into a tab delimitted file, later we can open 
   * it with Matlab. Use: plot_data5.m to plot the results.
   */
  ofstream log_file;
#ifdef _WIN32
  log_file.open("..\\bin\\log_file5.txt");
#else
  log_file.open("log_file5.txt");
#endif
  
  int n_states = 3;
  int n_outputs = 1;
  mat Q(n_states, n_states);
  mat R(n_outputs, n_outputs);
  
  double q = 0.1;
  double r = 0.1;
  
  Q << q*q << 0   << 0   << endr
    << 0   << q*q << 0   << endr
    << 0   << 0   << q*q << endr;
    
  R << r*r << endr;
  
  colvec x0(n_states);
  x0 << 0 << 0 << 1;
  
  colvec u;

  // No inputs
  u = u.zeros(); 
  
  MyEKF myekf;
  myekf.InitSystem(n_states, n_outputs, Q, R);
  myekf.InitSystemState(x0);
  
  for (int k = 0; k < 20; k ++) {
    myekf.EKalmanf(u);
    
    colvec *x = myekf.GetCurrentState();
    colvec *x_m = myekf.GetCurrentEstimatedState();
    colvec *z = myekf.GetCurrentOutput();
    
    log_file << k << '\t' << x->at(0,0) << '\t' << x_m->at(0,0)  
                  << '\t' << x->at(1,0) << '\t' << x_m->at(1,0)
                  << '\t' << x->at(2,0) << '\t' << x_m->at(2,0)
                  << '\n'; 
  }
  
  log_file.close();
  
  return 0;
}
