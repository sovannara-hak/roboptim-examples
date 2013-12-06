#include <roboptim/core.hh>
#include <stdio.h>
#include <roboptim/core/plugin/ipopt/ipopt.hh>
#include <boost/shared_ptr.hpp>

typedef roboptim::IpoptSolver::solver_t solver_t;

//Resolution of min 1/2 x^t A x + b^t x with x in [-4, +infinity]
//and x_1 in [-1.0, 0.7]
//    x_2 in [-1.3, 1.4]
//    x_3 in [-2.0, 2.1]
int main()
{
    //Definition of the quadratic function
    roboptim::Function::matrix_t A( 3,3 );
    roboptim::Function::vector_t b( 3 );

    A << 1,7,4,7,3,6,4,6,2;
    b << 1,3,4;

    roboptim::NumericQuadraticFunction f( A, b );

    //Testing f([1,1,1])
    roboptim::Function::vector_t x( 3 );
    x << 1,1,1;
    std::cout << "f(x) = " << f( x ) << std::endl;

    //Definition the constraint x_min < x < x_max
    roboptim::Function::vector_t offset( 3 );
    boost::shared_ptr< roboptim::IdentityFunction > c1(
            new roboptim::IdentityFunction( offset ) );

    //Definition of another constraint 0 < Ax +b < +infinity
    roboptim::Function::matrix_t Ac2( 3,3 );
    roboptim::Function::vector_t bc2( 3 );
    Ac2 << 2,3,1,7,1,0,3,5,2;
    bc2 << 4,3,4;
    boost::shared_ptr< roboptim::NumericLinearFunction > c2(
            new roboptim::NumericLinearFunction(Ac2, bc2) );
    
    //Test c([1,1,1])
    roboptim::IdentityFunction c1_el = *c1.get();
    std::cout << "c1(x) = [" << c1_el( x ) << "]" << std::endl;
    
    //Creation of the problem
    solver_t::problem_t pb ( f );
    std::cout << "problem input size : " << pb.function().inputSize() << std::endl;
    std::cout << "problem output size : " << pb.function().outputSize() << std::endl;

    //Set bounds for x in f(x)
    roboptim::Function::intervals_t function_arg_bounds;
    for( unsigned int i=0; i< pb.function().inputSize(); ++i ){
        roboptim::Function::interval_t x_bound = roboptim::Function::makeInterval( -4.0, roboptim::Function::infinity() );
        function_arg_bounds.push_back( x_bound );
    }

    pb.argumentBounds() = function_arg_bounds;

    //Set bounds for constraints: x_min and x_max
    roboptim::Function::intervals_t constraint_bounds;
    roboptim::Function::interval_t c1_0 = roboptim::Function::makeInterval( -1.0, 0.7 );
    roboptim::Function::interval_t c1_1 = roboptim::Function::makeInterval( -1.3, 1.4 );
    roboptim::Function::interval_t c1_2 = roboptim::Function::makeInterval( -2.0, 2.1 );
    constraint_bounds.push_back( c1_0 );
    constraint_bounds.push_back( c1_1 );
    constraint_bounds.push_back( c1_2 );

    //Set bounds for c2
    roboptim::Function::intervals_t c2_bounds;
    for( unsigned int i=0; i< pb.function().inputSize(); ++i ){
        roboptim::Function::interval_t c2_interval = roboptim::Function::makeLowerInterval( 0 );
        c2_bounds.push_back(c2_interval);
    }

    //Set scales for the problem
    solver_t::problem_t::scales_t scales(pb.function().inputSize(), 1.0);
    
    //Add constraint
    pb.addConstraint(
            boost::static_pointer_cast<
      roboptim::GenericLinearFunction<roboptim::EigenMatrixDense>  >
            (c1), constraint_bounds, scales );

    
    pb.addConstraint(
            boost::static_pointer_cast<
      roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> >
            (c2), c2_bounds, scales);
      
    //Set initial guess
    roboptim::NumericQuadraticFunction::argument_t x_init( 3 );
    x_init << 0.4,0.0,0.2;

    pb.startingPoint() = x_init;

    //Set path of the roboptim-core-plugin-ipopt.so so that libltdl finds the lib.
    //The path is detected in the CMakeLists.txt which will substitute the variable PLUGIN_PATH during the compilation
    //Those two lines can be omitted if the environment variable LD_LIBRARY_PATH contains the path of the plugin
    lt_dlinit();
    lt_dlsetsearchpath (PLUGIN_PATH);

    roboptim::SolverFactory<solver_t> factory ("ipopt", pb);
    solver_t& solver = factory ();

    solver_t::result_t res = solver.minimum ();

    std::cout << solver << std::endl;

    // Check if the minimization has succeed.
    if (res.which () != solver_t::SOLVER_VALUE)
    {
        std::cout << "A solution should have been found. Failing..."
            << std::endl
            << boost::get<roboptim::SolverError> (res).what ()
            << std::endl;
        return 0;
    } 

    // Get the result.
    roboptim::Result& result = boost::get<roboptim::Result> (res);
    std::cout << "A solution has been found: " << std::endl
              << result << std::endl;
    return 1;
}
