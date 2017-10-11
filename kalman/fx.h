/**
 * @file fx.h
 * @author Auralius Manurung
 * @date 28 Jan 2017
 * @brief A header file for a user defined mathematical function.
 *
 * @section DESCRIPTION
 * Example:
 * A function of vector, with 3 input parameters and 2 output parameters. 
 * \f[y = \begin{bmatrix} f_1(x_1,x_2,x_3) \\ f_2(x_1,x_2,x_3) \end{bmatrix} \f]
 * \f[Jac(y)=\begin{bmatrix} \frac{\partial f_1}{\partial x_1} \frac{\partial f_1}{\partial x_2} \frac{\partial f_1}{\partial x_3} \\ 
 *                           \frac{\partial f_2}{\partial x_1} \frac{\partial f_2}{\partial x_2} \frac{\partial f_2}{\partial x_3} 
 *           \end{bmatrix} \f] 
 *
 * \f[Hess(y(1)) =\begin{bmatrix} \frac{\partial^2 f_1}{\partial x_1^2}              \frac{\partial^2 f_1}{\partial x_1 \partial x_2}   \frac{\partial^2 f_1}{\partial x_1 \partial x_3}   \\
 *                                \frac{\partial^2 f_1}{\partial x_2 \partial x_1}   \frac{\partial^2 f_1}{\partial x_2^2}              \frac{\partial^2 f_1}{\partial x_2 \partial x_3}   \\
 *                                \frac{\partial^2 f_1}{\partial x_3 \partial x_1}   \frac{\partial^2 f_1}{\partial x_3 \partial x_2}   \frac{\partial^2 f_1}{\partial x_3^2}
 *                \end{bmatrix} \f]
 *
 * \f[Hess(y(2)) =\begin{bmatrix} \frac{\partial^2 f_2}{\partial x_1^2}              \frac{\partial^2 f_2}{\partial x_1 \partial x_2}   \frac{\partial^2 f_2}{\partial x_1 \partial x_3}   \\
 *                                \frac{\partial^2 f_2}{\partial x_2 \partial x_1}   \frac{\partial^2 f_2}{\partial x_2^2}              \frac{\partial^2 f_2}{\partial x_2 \partial x_3}   \\
 *                                \frac{\partial^2 f_2}{\partial x_3 \partial x_1}   \frac{\partial^2 f_2}{\partial x_3 \partial x_2}   \frac{\partial^2 f_2}{\partial x_3^2}
 *                \end{bmatrix} \f]
 *
 */

#ifndef FX_H
#define FX_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <assert.h>
#include <armadillo>

using namespace std;
using namespace arma;

class FX
{
public:
    /*!
     * \brief Constructor, create a mathematical function.
     * \param f Address to the user defined mathematical function. 
     */
    FX(colvec(*f) (colvec &x, colvec &some_constants));
    
    /*!
     * \brief Destructor, nothing happens here.
     */
    ~FX();

    /*!
     * \brief Calculate the Jacobian at certain inputs.
     * \param x Location where the Jacobian is computed.
     * \param some_constants Optional constants used in the function.
     * \return Jacobian at location x
     */
    mat JacobianAt(colvec &x, colvec &some_constants);
    
    /*!
     * \brief Calculate the Hessian, at certain inputs.
     * \param x Location where the Hessian is computed.
     * \param some_constants Optional constants used in the function.
     * \param i For a function of vector, do Hessian at i-th element of the vector.
     * \return Hessian at location x.
     */
    mat HessianAt(colvec &x, colvec &some_constants, int i);
     
    /*!
     * \brief Solve the function at certain inputs.
     * \param x Location where the function is solved.
     * \param some_constants Optional constants used in the function.
     * \return Result from solving the function.
     */
    colvec SolveAt(colvec &x, colvec &some_constants);

    /*!
     * \brief A very small number.
     * \param epsilon A very small number.
     */
    void SetEpsilon(double epsilon);

private: 
    colvec (*F_) (colvec &x, colvec &some_constants); ///< Remember the address of the callback function.

    double Epsilon_; ///< Number of the states
};

#endif
