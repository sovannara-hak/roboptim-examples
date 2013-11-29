#include <roboptim/core.hh>
#include <stdio.h>
#include <roboptim/core/plugin/ipopt/ipopt.hh>
#include <boost/shared_ptr.hpp>

typedef roboptim::IpoptSolver::solver_t solver_t;

int main()
{
    roboptim::Function::matrix_t A( 3,3 );
    roboptim::Function::vector_t b( 3 );

    A << 1,7,4,7,3,6,4,6,2;
    b << 1,3,4;

    roboptim::NumericQuadraticFunction f( A, b );

    roboptim::Function::vector_t x( 3 );
    x << 1,1,1;
    std::cout << "f(x) = " << f( x ) << std::endl;

    roboptim::Function::vector_t offset( 3 );

    boost::shared_ptr< roboptim::IdentityFunction > c1(
            new roboptim::IdentityFunction( offset ) );
    
    roboptim::IdentityFunction c1_el = *c1.get();
    std::cout << "c1(x) = [" << c1_el( x ) << "]" << std::endl;
    
    solver_t::problem_t pb ( f );
    std::cout << "problem input size : " << pb.function().inputSize() << std::endl;
    std::cout << "problem output size : " << pb.function().outputSize() << std::endl;

    roboptim::Function::intervals_t function_bounds;
    for( unsigned int i=0; i< pb.function().inputSize(); ++i ){
        roboptim::Function::interval_t x_bound = roboptim::Function::makeInterval( -4.0, roboptim::Function::infinity() );
        function_bounds.push_back( x_bound );
    }

    pb.argumentBounds() = function_bounds;

    roboptim::Function::intervals_t constraint_bounds;
    roboptim::Function::interval_t c1_0 = roboptim::Function::makeInterval( -1.0, 0.7 );
    roboptim::Function::interval_t c1_1 = roboptim::Function::makeInterval( -1.3, 1.4 );
    roboptim::Function::interval_t c1_2 = roboptim::Function::makeInterval( -2.0, 2.1 );
    constraint_bounds.push_back( c1_0 );
    constraint_bounds.push_back( c1_1 );
    constraint_bounds.push_back( c1_2 );

    solver_t::problem_t::scales_t scales(pb.function().inputSize(), 1.0);
    
    //pb.addConstraint( c1, constraint_bounds, scales );

    return 1;
}
