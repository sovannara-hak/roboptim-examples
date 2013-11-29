#include <roboptim/core.hh>
#include <roboptim/core/function.hh>
#include <stdio.h>

int main()
{
    roboptim::Function::matrix_t A(3,3);
    roboptim::Function::vector_t b(3);

    A << 1,7,4,7,3,6,4,6,2;
    b << 1,3,4;
    roboptim::NumericQuadraticFunction F(A, b);

    std::cout << A << std::endl;

    std::cout << b << std::endl;
    
    return 1;
}
