#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
// 前轮到重心的距离
const double Lf = 2.67;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(const Eigen::VectorXd &state, 
                            const Eigen::VectorXd &coeffs);
};

// 每个损失量的权重
const int cte_cost_weight = 2000;
const int epsi_cost_weight = 2000;
const int v_cost_weight = 1;
const int delta_cost_weight = 10;
const int a_cost_weight = 10;
const int delta_change_cost_weight = 100;
const int a_change_cost_weight = 10;

#endif  // MPC_H
