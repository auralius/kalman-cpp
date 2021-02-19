/**
 * @file kf.h
 * @author Auralius Manurung
 * @date 18 Apr 2015
 * @brief Header file for the Kalman filter, for a linear system.
 * 
 * Define a system:
 * $$x_k = Ax_{k-1} + Bu_{k-1} + v_{k-1}$$
 * $$z_k = Hx_k + w_k\$$
 * where:\n
 * $v$ is the process noise (Gaussian with covariance Q)\n  
 * $w$ is the measurement noise (Gaussian with covariance R)\n 
 * $A$ is the system matrix\n
 * $B$ is the input matrix\n
 * $H$ is the output matrix\n
 * $x$ is the state vector\n
 * $z$ is the output vector\n
 * $u$ is the input vector\n
 */

#ifndef KF_H
#define KF_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <assert.h>
#include <armadillo>

using namespace std;
using namespace arma;

/*!
 * @brief Implemetation of the Kalman filter. 
 */
class KF {
public:
  KF();
  ~KF();
  
  /*!
   * @brief Define the system.
   * @param A System matrix
   * @param B Input matrix
   * @param H Output matrix
   * @param Q Process noise covariance
   * @param R Measurement noise covariance
   */
  void InitSystem (const mat& A, const mat& B, const mat& H, const mat& Q, const mat& R);
  
  /*!
   * @brief Initialize the system states.
   * Must be called after InitSystem.
   * If not, called, system states are initialized to zero.
   * @param x0 Inital value for the system state
   */
  void InitSystemState(const colvec& x0);
  
  /*!
   * @brief Initialize the state covariance.
   * Must be called after InitSystem.
   * If not called, covariance state is Initialized to an identity matrix.
   * @param P0 Inital value for the state covariance
   */
  void InitStateCovariance(const mat& P0);
  
  /*!
   * @brief Do Kalman filter iteration step-by-step while simulating the system. 
   * Simulating the system is done to calculate system states and outputs.
   * @param u tThe applied input to the system
   */
  void Kalmanf(const colvec& u);
  
  /*!
   * @brief Do Kalman filter iteration step-by-step without simulating the system. 
   * Use this if measurement is available and simulating the system is unnecessary.
   * Here, true system states and system outputs do not matter. 
   * The only thing that matters is the estimated states. 
   * @param z The values of the output from measurement
   * @param u The applied input to the system
   */
  void Kalmanf(const colvec& z, const colvec& u);
 
 /*!
  * @brief Get current simulated true state.
  * @return Current simulated state $x_k$
  */
  colvec* GetCurrentState();
  
   /*!
  * @brief Get current simulated true output.
  * This is analogous to the measurements.
  * @return Current simulated output $z_k$
  */
  colvec* GetCurrentOutput();
  
 /*!
  * @brief Get current estimated state.
  * @return Current estimated state $\hat{x}_k$
  */
  colvec* GetCurrentEstimatedState();
  
   /*!
  * @brief Get current estimated output.
  * This is analogous to the filtered measurements.
  * @return Current estimated output $\hat{z}_k$
  */
  colvec* GetCurrentEstimatedOutput();
  
private:
  
  mat A_;      ///< System matrix
  mat B_;      ///< Input matrix
  mat H_;      ///< Output matrix
  mat Q_;      ///< Process noise covariance
  mat R_;      ///< Measurement noise covariance
  colvec v_;   ///< Gaussian process noise
  colvec w_;   ///< Gaussian measurement noise
  
  mat sqrt_Q_; ///< Process noise stdev
  mat sqrt_R_; ///< Measurement noise stdev
  
  colvec x_;   ///< State vector
  colvec z_;   ///< Output matrix
 
  colvec x_m_; ///< State vector after measurement update
  colvec x_p_; ///< State vector after a priori update
  
  mat P_p_;    ///< State covariance after a priori update
  mat P_m_;    ///< State covariance after measurement update
  
  colvec z_m_; ///< Estimated output
};

#endif
