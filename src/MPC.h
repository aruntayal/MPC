#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


// weights for cost computations
const double W_CTE = 1500.0;
const double W_EPS = 1500.0;
const double W_V = 1.0;
const double W_DELTA = 10.0;
const double W_A = 10.0;
const double W_DDELTA = 150.0; 
const double W_DA = 15.0; 

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

    vector<double> mpc_x;
    vector<double> mpc_y;
};

#endif /* MPC_H */
