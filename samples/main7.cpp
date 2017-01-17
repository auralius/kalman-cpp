/**
 * @file main7.cpp
 * @author Auralius Manurung
 * @date 16 Jan 2017
 * 
 * @brief Example for the Kalman filter.
 *
 * An example on using Kalman fiter for smoothing measurement data.
 * Assume we have a measurment data. 
 * We will use the second scenario of using Kalman filter.
 * In this scenario, there is no need for the Kalman precdure to simulate the system.
 * We then tune Q and R to have the best filtering performance.
 */

#include "kf.h"
#include "../ext/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char** argv)
  {      
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
    
    /*
     * For plotting with matplotlibcpp
     */
    int N = 500;
    std::vector<double> k_plot(N), noisy_plot(500), smooth_plot(N);
  
    for (int k = 0; k < N; k++) {
        
      // Assume we have a noisy signal that is acquired from a measurement tool
      double w = 2; // Stdev of the noise, in reality we don't know this
      colvec z_measurment(1);
      z_measurment.randn(1);
      z_measurment = z_measurment * w + 12.0;
      
      // Put the z_measurment to the Kalman filter
      kalman.Kalmanf(z_measurment, u);
      
      colvec *z_m = kalman.GetCurrentEstimatedOutput();
      colvec *x_m = kalman.GetCurrentEstimatedState();
      
      k_plot.at(k) = k;
      noisy_plot.at(k) =  z_measurment.at(0,0);
      smooth_plot.at(k) = z_m->at(0,0);
    }
    
    plt::named_plot("Output (measurement)", k_plot, noisy_plot, "r+");
    plt::named_plot("Estimated output", k_plot, smooth_plot);
    plt::legend();
    plt::show();
    
    return 0;
  }