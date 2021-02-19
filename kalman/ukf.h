/**
 * @file ukf.h
 * @author Auralius Manurung
 * @date 13 Peb 2021
 * @brief Header file for the  Unscented Kalman filter (UKF).
 * 
 * @section DESCRIPTION
 * The implementation is based on the paper: Wan, E. A., & Van Der Merwe, 
 * R. (2006). The unscented Kalman filter for nonlinear estimation. 
 * Proceedings of the IEEE 2000 Adaptive Systems for Signal Processing, 
 * Communications, and Control Symposium (Cat. No.00EX373), 31(2), 
 * 153–158. https://doi.org/10.1109/ASSPCC.2000.882463\n\n
 * Define a non-linear discrete-time process: 
 * \f[x_k = f(x_{k-1}, u_{k-1}) + v_{k-1}\f]
 * \f[z_k = h(x_k) + w_k\f]
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
 * @brief Implemetation of the Unscented Kalman filter. 
 * This class needs to be derived.
 */
class UKF {
public:  
  /*!
   * \brief Constructor, nothing happens here.
   */
  UKF();
  
  /*!
   * \brief Destructur, nothing happens here.
   */
  ~UKF();
  
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
   */
  virtual colvec f(const colvec &x, const colvec &u);
  
  /*!
   * \brief Define the output model of your system.
   * @param x System states
   */
  virtual colvec h(const colvec &x);
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
   */
  void UKalmanf(const colvec& u);
  
  /*!
   * \brief Do the extended Kalman iteration step-by-step without simulating the system. 
   * Use this if measurement is available and simulating the system is unnecessary.
   * Here, true system states and system outputs do not matter. 
   * The only thing that matters is the estimated states.
   * @param z The measurement outputs, this is a returned value
   * @param u The applied input to the system
   */
  void UKalmanf(const colvec& z, const colvec& u);
  
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

  mat Q_;          ///< Process noise covariance
  mat R_;          ///< Measurement noise covariance
  colvec v_;       ///< Gaussian process noise
  colvec w_;       ///< Gaussian measurement noise
  
  mat sqrt_Q_;     ///< Process noise stdev
  mat sqrt_R_;     ///< Measurement noise stdev
  
  mat P_;          ///< Covariance
  colvec x_m_;     ///< Estimated state
  colvec z_m_;     ///< Estimated output

protected:
  
  int nStates_;   ///< Number of the states
  int nOutputs_;  ///< Number of outputs
  
  colvec x_;      ///< State vector, by simulaiton, true value 
  colvec z_;      ///< Output matrix, by simulation, true value
};


#endif
