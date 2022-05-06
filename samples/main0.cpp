/**
 * This is the smole test that comes with the Armadillo lib.
 * The purpose is to ensure that LAPACK is working well together with the Armadillo.
 * Make sure you can runs this test without any problems!
 */

#define ARMA_USE_LAPACK // This definition must be before the include to the armadillo

#include <iostream>
#include <armadillo>

using namespace arma;

int
main()
{
    std::cout << "*** smoke test start" << std::endl;

    uword N = 5;

    mat A = reshape(regspace(1, N * N), N, N);

    A.diag() += randu<vec>(N);

    mat B;

    bool status = expmat(B, A);

    A.print("A:");
    B.print("B:");

    std::cout << ((status) ? "*** smoke test okay" : "*** smoke test failed") << std::endl;

    return (status) ? 0 : -1;
}
