/**
 * @file fun.h
 * @author Auralius Manurung
 * @date 28 Jan 2017
 * @brief A header file for a user defined mathematical function.
 *
 * @section DESCRIPTION 
 *
 s*/

#ifndef FUN_H
#define FUNs_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <assert.h>
#include <armadillo>

using namespace std;
using namespace arma;

class FUN
{
public:
    FUN(colvec(*f) (colvec &x, colvec &some_constants));
    ~FUN();

    mat JacobianAt(colvec &x, colvec &some_constants);
    
    mat HessianAt(colvec &x, colvec &some_constants, int i);

    colvec SolveAt(colvec &x, colvec &some_constants);

    void SetEpsilon(double epsilon);

private:  
    colvec (*F_) (colvec &x, colvec &some_constants);

    double Epsilon_;
};

#endif
