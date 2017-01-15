/**
 * @file ekf.h
 * @author Auralius Manurung
 * @date 18 Apr 2015
 * @brief Header file for the extended Kalman filter implementation, for a nonlinear system.
 * 
 * @section DESCRIPTION
 * Define a non-linear discrete-time process: 
 * \f[x_k = f(x_{k-1}, u_{k-1}, k) + v_{k-1}\f]
 * \f[z_k = h(x_k, u_k, k) + w_k\f]
 * where:\n 
 * \f$f\f$ is the dynamic model of the system\n
 * \f$h\f$ is the measurement model of the system\n
 * \f$v\f$ is the process noise (Gaussian with covariance Q)\n  
 * \f$w\f$ is the measurement noise (Gaussian with covariance R)\n 
 * \f$x\f$ is the state vector\n
 * \f$z\f$ is the output vector\n
 * \f$u\f$ is the input vector\n
 */

#ifndef EKF_H
#define EKF_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <assert.h>
#include <armadillo>

using namespace std;
using namespace arma;

/*!
 * @brief Implemetation of the extended Kalman filter. 
 * This class needs to be derived.
 */
class EKF {
public:  
  /*!
   * \brief Constructor, nothing happens here.
   */
  EKF();
  
  /*!
   * \brief Destructur, nothing happens here.
   */
  ~EKF();
  
  /*!
   * \brief Tell me how many states and outputs you have!
   * @param n_states Number of the elements on the input vector x
   * @param n_outputs Number of the elements on the otput vector z
   * @param Q Process noise covariance
   * @param R Measurement noise covariance
   */
  void InitSystem(int n_states, int n_outputs, const mat& Q, const mat& R);
  /*!
   * \brief Define model of your system.
   * @param x System states
   * @param u System inputs
   * @param k k-th iteration
   */
  virtual colvec f(const colvec &x, const colvec &u, const int k);
  
  /*!
   * \brief Define the output model of your system.
   * @param x System states
   * @param u Input vector
   * @param k k-th iteration
   */
  virtual colvec h(const colvec &x, const colvec &u, const int k);
  
  /*!
   * \brief Initialize the system states.
   * Must be called after InitSystem.
   * If not called, system state is initialized to zero.
   * @param x0 Inital value for the system state
   */
  void InitSystemState(const colvec& x0);
  
  /*!
   * \brief Initialize the system state covariance.
   * Must be called after InitSystem.
   * If not called, state covariance is initialized to an identity matrix.
   * @param P0 Inital value for the state covariance
   */
  void InitSystemStateCovariance(const mat& P0);
 
  /*!
   * \brief Do the extended Kalman iteration step-by-step while simulating the system. 
   * Simulating the system is done to calculate system states and outputs.
   * @param u The applied input to the system
   * @param k k-th iteration
   */
  void EKalmanf(const colvec& u, const int k);
  
  /*!
   * \brief Do the extended Kalman iteration step-by-step without simulating the system. 
   * Use this if measurement is available and simulating the system is unnecessary.
   * Here, true system states and system outputs do not matter. 
   * The only thing that matters is the estimated states.
   * @param z The measurement outputs, this is a returned value
   * @param u The applied input to the system
   * @param k k-th iteration
   */
  void EKalmanf(const colvec& z, const colvec& u, const int k);
  
 /*!
  * @brief Get current simulated true state.
  * @return Current simulated state of the system \f$x_k\f$
  */
  colvec* GetCurrentState();
  
 /*!
  * @brief Get current simulated true output.
  * This is analogous to the measurements.
  * @return Current simulated output \f$z_k\f$
  */
  colvec* GetCurrentOutput();
  
 /*!
  * @brief Get current estimated state.
  * This is analogous to the filtered measurements.
  * @return Current estimated state \f$\hat{x}_k\f$
  */
  colvec* GetCurrentEstimatedState();
  
 /*!
  * @brief Get current estimated output.
  * @return Current estimated output \f$\hat{z}_k\f$
  */
  colvec* GetCurrentEstimatedOutput();
  
private:
  /*!
   * \brief Compute the Jacobian of f numerically using  a  small  
   * finite-difference perturbation magnitude. 
   * @param x System states
   * @param u Input vector
   * @param k k-th iteration
   */
  void CalcF(const colvec &x, const colvec &u, const int k);
  
  /*!
   * \brief Compute the Jacobian of h numerically using  a  small  
   * finite-difference perturbation magnitude.
   * @param x System states
   * @param u Input vector
   * @param k k-th iteration 
   */
  void CalcH(const colvec &x, const colvec &u, const int k);

  mat F_;          ///< Jacobian of F	
  mat H_;          ///< Jacobian of H
  mat Q_;          ///< Process noise covariance
  mat R_;          ///< Measurement noise covariance
  colvec v_;       ///< Gaussian process noise
  colvec w_;       ///< Gaussian measurement noise
  
  mat sqrt_Q_;     ///< Process noise stdev
  mat sqrt_R_;     ///< Measurement noise stdev
  

 
  colvec x_m_;     ///< State vector after measurement update
  colvec x_p_;     ///< State vector after a priori update
  
  mat P_p_;        ///< State covariance after a priori update
  mat P_m_;        ///< State covariance after measurement update
  
  colvec z_m_; ///< Estimated output
  
  double epsilon_; ///< Very small number
  
protected:
  
  int nStates_;   ///< Number of the states
  int nOutputs_;  ///< Number of outputs
  
  colvec x_;      ///< State vector
  colvec z_;      ///< Output matrix
};


#endif
