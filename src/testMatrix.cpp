#include <roboptim/core.hh>
#include <stdio.h>
#include <roboptim/core/plugin/ipopt/ipopt.hh>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

int main()
{
    //Definition of the quadratic function
    roboptim::Function::matrix_t A( 3,3 );
    roboptim::Function::vector_t b( 3 );

    A << 1,7,4,7,3,6,4,6,2;
    b << 1,3,4;

    Eigen::MatrixXd Ae(3,3);
    Ae << 1,2,3,4,5,6,7,8,9;

    Eigen::MatrixXd C(3, 6);
    C << Ae, A;

    std::cout << C << std::endl;

    return 1;
}
