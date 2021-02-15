/**
* @file main10.cpp
* @author Auralius Manurung
* @date 14 Peb 2021
*
* @brief An example for an Unscented Kalman filter (UKF).
*
* This example is taken from <a href="https://www.mathworks.com/matlabcentral/fileexchange/18217-learning-the-unscented-kalman-filter">here</a>,
* "Learning the Unscented Kalman Filter" by Yi Cao.
*/

#include <fstream>

#include "ukf.h"

/// @cond DEV
/*
* Class UKF needs to be derived, two virtual functions are provided in
* which system model and output model are described.
*/
class MyUKF : public UKF
{
public:
    virtual colvec f(const colvec& x, const colvec& u) {
        colvec xk(nStates_);
        xk.at(0) = x(1);
        xk.at(1) = x(2);
        xk.at(2) = 0.05*x(0)*(x(1)+x(2));
        //xk.print("xk=");
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
    * it with Matlab. Use: plot_data10.m to plot the results.
    */
    ofstream log_file;
#ifdef _WIN32
    log_file.open("..\\bin\\log_file10.txt");
#else
    log_file.open("log_file10.txt");
#endif

    mat Q(3, 3);
    mat R(1, 1);


    MyUKF myukf;

    double r = 0.1;
    double q = 0.1;
    
    Q = eye(3,3)*q*q;
    R << r*r << endr;

    colvec x0(3);
    x0 << 0 << 0 << 1;

    mat P0 = eye<mat>(3, 3);
    P0 =  P0;

    colvec u;

    // No inputs
    u = u.zeros();

    myukf.InitSystem(3, 1, Q, R);
    myukf.InitSystemState(x0);
    myukf.InitSystemStateCovariance(P0);

    for (int k = 0; k < 20; k++) {
        myukf.UKalmanf(u);

        colvec *x = myukf.GetCurrentState();
        colvec *x_m = myukf.GetCurrentEstimatedState();
        colvec *z = myukf.GetCurrentOutput();
        colvec *z_m = myukf.GetCurrentEstimatedOutput();

        log_file << k << '\t' << x->at(0, 0) << '\t' << x_m->at(0, 0)
            << '\t' << x->at(1, 0) << '\t' << x_m->at(1, 0)
            << '\t' << x->at(2, 0) << '\t' << x_m->at(2, 0)
            << '\t' << z->at(0, 0) << '\t' << z_m->at(0, 0) << '\t'
            << '\n';
    }

    log_file.close();

    return 0;
}