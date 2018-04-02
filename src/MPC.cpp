#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//

size_t N = 10; // how many states we "lookahead" in the future
const double DT = 0.1; // how much time we expect environment changes

const double Lf = 2.67; //Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
const double VELOCITY_MAX = 80.0; 

const int NUMBER_OF_STATES = 6; // px, py, psi, v, cte, epsi
const int NUMBER_OF_ACTUATIONS = 2; // steering angle, acceleration

//const int NX =  // number of state + actuation variables
//const int NG = N * NUMBER_OF_STATES; // number of constraints

const int ID_FIRST_px = 0;
const int ID_FIRST_py = ID_FIRST_px + N;
const int ID_FIRST_psi = ID_FIRST_py + N;
const int ID_FIRST_v = ID_FIRST_psi + N;
const int ID_FIRST_cte = ID_FIRST_v + N;
const int ID_FIRST_epsi = ID_FIRST_cte + N;
const int ID_FIRST_delta = ID_FIRST_epsi + N;
const int ID_FIRST_a = ID_FIRST_delta + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    fg[0] = 0;
    for (int i = 0; i < N; i++) {
      fg[0] += W_CTE * CppAD::pow(vars[ID_FIRST_cte + i], 2);
      fg[0] += W_EPS * CppAD::pow(vars[ID_FIRST_epsi + i], 2);
      fg[0] += W_V * CppAD::pow(vars[ID_FIRST_v + i] - VELOCITY_MAX, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += W_DELTA * CppAD::pow(vars[ID_FIRST_delta + i], 2);
      fg[0] += W_A * CppAD::pow(vars[ID_FIRST_a + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += W_DDELTA * CppAD::pow(vars[ID_FIRST_delta + i + 1] - vars[ID_FIRST_delta + i], 2);
      fg[0] += W_DA * CppAD::pow(vars[ID_FIRST_a + i + 1] - vars[ID_FIRST_a + i], 2);
    }


    //CONSTRAINTS

      // given state 
      fg[ID_FIRST_px + 1] = vars[ID_FIRST_px];
      fg[ID_FIRST_py + 1] = vars[ID_FIRST_py];
      fg[ID_FIRST_psi + 1] = vars[ID_FIRST_psi];
      fg[ID_FIRST_v + 1] = vars[ID_FIRST_v];
      fg[ID_FIRST_cte + 1] = vars[ID_FIRST_cte];
      fg[ID_FIRST_epsi + 1] = vars[ID_FIRST_epsi];

       
    // constraints based on our kinematic model
      for (int t = 0; t < N - 1; ++t) {
       // The state at time t.
      AD<double> x0 = vars[ID_FIRST_px + t] ;
      AD<double> y0 = vars[ID_FIRST_py + t ];
      AD<double> psi0 = vars[ID_FIRST_psi + t ];
      AD<double> v0 = vars[ID_FIRST_v + t ];
      AD<double> cte0 = vars[ID_FIRST_cte + t ];
      AD<double> epsi0 = vars[ID_FIRST_epsi + t ];


      // The state at time t+1 .
      AD<double> x1 = vars[ID_FIRST_px + t + 1];
      AD<double> y1 = vars[ID_FIRST_py + t + 1];
      AD<double> psi1 = vars[ID_FIRST_psi + t + 1];
      AD<double> v1 = vars[ID_FIRST_v + t + 1];
      AD<double> cte1 = vars[ID_FIRST_cte + t + 1];
      AD<double> epsi1 = vars[ID_FIRST_epsi + t + 1];

           // Only consider the actuation at time t.
      AD<double> delta0 = vars[ID_FIRST_delta + t];
      AD<double> a0 = vars[ID_FIRST_a + t];

     
        // desired py and psi
        const auto py_desired = coeffs[3] * x0 * x0 * x0 + coeffs[2] * x0 * x0 + coeffs[1] * x0 + coeffs[0];
        const auto psi_desired = CppAD::atan(3.0 * coeffs[3] * x0 * x0 + 2.0 * coeffs[2] * x0 + coeffs[1]);

        // relationship of current state + actuations and next state
        // based on our kinematic model
        const auto px1_f = x0 + v0 * CppAD::cos(psi0) * DT;
        const auto py1_f = y0 + v0 * CppAD::sin(psi0) * DT;
        const auto psi1_f = psi0 + v0 * (-delta0) / Lf * DT;
        const auto v1_f = v0 + a0 * DT;
        const auto cte1_f = py_desired - y0 + v0 * CppAD::sin(epsi0) * DT;
        const auto epsi1_f = psi0 - psi_desired + v0 * (-delta0) / Lf * DT;

        // store the constraint expression of two consecutive states
        fg[ID_FIRST_px + t + 2] = x1 - px1_f;
        fg[ID_FIRST_py + t + 2] = y1 - py1_f;
        fg[ID_FIRST_psi + t + 2] = psi1 - psi1_f;
        fg[ID_FIRST_v + t + 2] = v1 - v1_f;
        fg[ID_FIRST_cte + t + 2] = cte1 - cte1_f;
        fg[ID_FIRST_epsi + t + 2] = epsi1 - epsi1_f;

    }
    //std::cout<<"\n operator() complete";



  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
 // size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

   const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars =  N * NUMBER_OF_STATES + (N - 1) * NUMBER_OF_ACTUATIONS;;
  // TODO: Set the number of constraints
  size_t n_constraints = N * NUMBER_OF_STATES;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[ID_FIRST_px] = x;
  vars[ID_FIRST_py] = y;
  vars[ID_FIRST_psi] = psi;
  vars[ID_FIRST_v] = v;
  vars[ID_FIRST_cte] = cte;
  vars[ID_FIRST_epsi] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.


  for (int i = 0; i < ID_FIRST_delta; i++) {
    vars_lowerbound[i] = -1.0e10;
    vars_upperbound[i] = 1.0e10;
  }
  
  for (int i = ID_FIRST_delta; i < ID_FIRST_a; i++) {
    vars_lowerbound[i] = -0.75;
    vars_upperbound[i] = 0.75;
  }
  // Acceleration/decceleration upper and lower limits
  for (int i = ID_FIRST_a; i < n_vars; i++) {
    vars_lowerbound[i] = -0.5;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[ID_FIRST_px] = x;
  constraints_lowerbound[ID_FIRST_py] = y;
  constraints_lowerbound[ID_FIRST_psi] = psi;
  constraints_lowerbound[ID_FIRST_v] = v;
  constraints_lowerbound[ID_FIRST_cte] = cte;
  constraints_lowerbound[ID_FIRST_epsi] = epsi;
  

  constraints_upperbound[ID_FIRST_px] = x;
  constraints_upperbound[ID_FIRST_py] = y;
  constraints_upperbound[ID_FIRST_psi] = psi;
  constraints_upperbound[ID_FIRST_v] = v;
  constraints_upperbound[ID_FIRST_cte] = cte;
  constraints_upperbound[ID_FIRST_epsi] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          1.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

 // Cost
  auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  if (ok) {
    std::cout << "OK! Cost:" << cost << std::endl;
  } else {
    std::cout << "SOMETHING IS WRONG!" << cost << std::endl;
  }

 

  this->mpc_x = {};
  this->mpc_y = {};
  //std::cout<<"\n initialized vector";
  for (int i = 0; i < N; i++) {
    this->mpc_x.push_back(solution.x[ID_FIRST_px + i]);
    this->mpc_y.push_back(solution.x[ID_FIRST_py + i]);
  }
 // std::cout<<"\n Push back to vector mpc_x";
  vector<double> result;
  result.push_back(solution.x[ID_FIRST_delta]);
  result.push_back(solution.x[ID_FIRST_a]);
  //std::cout<<"\n Push back to vector result";
  return result;
}
