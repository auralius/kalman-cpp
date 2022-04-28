/**
 * @file main3.cpp
 * @author Auralius Manurung
 * @date 18 Apr 2015
 * 
 * @brief Example for the Kalman filter.
 * 
 * @section DESCRIPTION
 * A system in x-y plane, a kinematic system, with position and velocity as the states.
 * The system has 4 input states (\f$x\f$, \f$y\f$, \f$\dot{x}\f$, \f$\dot{y}\f$).
 * and 2 output states (\f$x\f$, \f$y\f$).
 */

#include <fstream>

#include "kf.h"


int main(int argc, char** argv)
  {
    /* 
     * Log the result into a tab delimitted file, later we can open 
     * it with Matlab. Use: plot_data3.m to plot the results.
     */
    ofstream log_file;
#ifdef _WIN32
    log_file.open("..\\bin\\log_file3.txt");
#else
    log_file.open("log_file3.txt");
#endif
   
    /*

     */
    mat A(4,4), B(4,1), H(2,4), Q(4,4), R(2,2);
    
    A << 1 << 0 << 1 << 0 << endr
      << 0 << 1 << 0 << 1 << endr
      << 0 << 0 << 1 << 0 << endr
      << 0 << 0 << 0 << 1 << endr;
    
    B = B.zeros();
    
    H << 1 << 0 << 0 << 0 << endr
      << 0 << 1 << 0 << 0 << endr;
    
    Q = Q.eye();
      
    R = 10 * R.eye();
    
    colvec x0(4);
    x0 << 10 << 10 << 1 << 0;
    
    mat P0(4,4);
    P0 = 10 * P0.eye();
      
    KF kalman;
    kalman.InitSystem(A, B, H, Q, R);
    kalman.InitSystemState(x0);
    kalman.InitStateCovariance(P0);
    
    colvec z(2);
    colvec x(4), x_m(4);
    colvec u(1);
    
    // No inputs, system subjects only to random perturbation
    u = u.zeros(); 
  
    for (int i = 0; i < 100 ; i ++) {  
      kalman.Kalmanf(u);
      
      colvec *x = kalman.GetCurrentState();
      colvec *z = kalman.GetCurrentOutput();
      colvec *x_m = kalman.GetCurrentEstimatedState();
      
      log_file << x->at(0,0) << '\t' << x->at(1,0) << '\t' << x->at(2,0) << '\t' << x->at(3,0) << '\t' 
               << x_m->at(0,0) << '\t' << x_m->at(1,0) << '\t' << x_m->at(2,0) << '\t' << x_m->at(3,0) << '\t' 
               << z->at(0,0) << '\t' << z->at(1,0)
               << '\n';
    }
    
    log_file.close();
    
    return 0;
  }
