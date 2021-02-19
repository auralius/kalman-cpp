/**
 * @file main7.cpp
 * @author Auralius Manurung
 * @date 16 Jan 2017
 * 
 * @brief An example of a Kalman filter practical application.
 *
 * An example on using Kalman fiter for smoothing measurement data.
 * Assume we have a measurment data. 
 * We will use the second scenario of using Kalman filter.
 * In this scenario, there is no need for the Kalman precdure to simulate the system.
 * We then tune Q and R to have the best filtering performance.
 */

#include "kf.h"

int main(int argc, char** argv)
  {    
  /* 
   * Log the result into a tab delimitted file, later we can open 
   * it with Matlab. Use: plot_data7.m to plot the results.
   */
  ofstream log_file;
#ifdef _WIN32
  log_file.open("..\\bin\\log_file7.txt");
#else
  log_file.open("log_file7.txt");
#endif
  
    /*
     * Define the system and initialize the Kalman filter
     */
    mat A(1,1), B(1,1), H(1,1), Q(1,1), R(1,1);
    colvec u(1);
    colvec x0(1);  
    
    A << 1;
    B << 1;
    Q << 0.01; // Heuristic tuning parameter
    H << 1;
    R << 1;    // Heuristic tuning parameter
    u << 0;
    
    KF kalman;
    kalman.InitSystem(A, B, H, Q, R);
    
    int N = 500;
  
    for (int k = 0; k < N; k++) {
        
      // Assume we have a noisy signal that is acquired from a measurement tool
      double w = 2; // Stdev of the noise, in reality we don't know this
      colvec z_measurement(1);
      z_measurement.randn(1);
      z_measurement = z_measurement * w + 12.0;
      
      // Put the z_measurment to the Kalman filter
      kalman.Kalmanf(z_measurement, u);
      
      colvec *z_m = kalman.GetCurrentEstimatedOutput();
      colvec *x_m = kalman.GetCurrentEstimatedState();
      
      log_file << k << '\t' << z_measurement.at(0,0) << '\t' << z_m->at(0,0)  
               << '\n'; 
    }

    return 0;
  }
