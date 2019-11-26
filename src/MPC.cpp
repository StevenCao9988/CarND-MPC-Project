#include "MPC.h"
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;
typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

ADvector Vehicle_Mode(const ADvector& state, const ADvector& actuators, AD<double> dt);  // ����ģ�ͺ��� Ԥ�⳵��״̬
AD<double> polyeval_AD(const VectorXd& coeffs, AD<double> x);                            // ����ʽ����

/**
 * TODO: Set the timestep length and duration
 */
size_t N = 10;
double dt = 0.1;

// Set desired speed for the cost function (i.e. max speed)
const double ref_v = 120;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) {this->coeffs = coeffs;}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  /** ����ģ�ͺ���
 * ���복������state��actuators��dt
 * �������Ԥ��״̬ result
 * const ADvector& state ����������������޸�state��ֵ
 */
  ADvector Vehicle_Mode(const ADvector& state,
	  const ADvector& actuators, double dt)
  {
	  ADvector next_state(state.size());
	  AD<double> x;
	  AD<double> y;
	  AD<double> psi;
	  AD<double> v;
	  AD<double> delta;
	  AD<double> a;

	  x = state[0];
	  y = state[1];
	  psi = state[2];
	  v = state[3];

	  delta = actuators[0];
	  a = actuators[1];

	  //std::cout << "delta " << delta << std::endl;
	  //std::cout << "v " << v << std::endl;
	  x = x + v * cos(psi) * dt;
	  y = y + v * sin(psi) * dt;
	  psi = psi - v * delta * dt / Lf;
	  v = v + a * dt;
	  next_state[0] = x;
	  next_state[1] = y;
	  next_state[2] = psi;
	  next_state[3] = v;

	  return next_state;
  }

  // Evaluate a polynomial.
  AD<double> polyeval_AD(const VectorXd& coeffs, AD<double> x) {
	  AD<double> result = 0.0;
	  for (unsigned  i = 0; i < coeffs.size(); i++) {
		  result += coeffs[i] * CppAD::pow(x, i);
	  }
	  return result;
  }

  void operator()(ADvector& fg, const ADvector& vars) {
    /**
     * TODO: implement MPC
     * `fg` is a vector of the cost constraints, `vars` is a vector of variable 
     *   values (state & actuators)
     * NOTE: You'll probably go back and forth between this function and
     *   the Solver function below.
     */
	 // Weights for how "important" each cost is - can be tuned
	  const int cte_cost_weight = 2000;
	  const int epsi_cost_weight = 2000;
	  const int v_cost_weight = 1;
	  const int delta_cost_weight = 10;
	  const int a_cost_weight = 150;
	  const int delta_change_cost_weight = 200;
	  const int a_change_cost_weight = 100;

	  fg[0] = 0;  // ��ʧ����
	  for (unsigned i = 0; i <= N - 1; i++)   // ����Ԥ��10��
	  {
		  // ��i�����Ԥ��λ��
		  AD<double> x = vars[i * 6];
		  AD<double> y = vars[i * 6 + 1];
		  AD<double> psi = vars[i * 6 + 2];
		  AD<double> v = vars[i * 6 + 3];

		  // �� i �������ʵλ��
		  AD<double> y_fit = coeffs[0] * CppAD::pow(x, 0) + coeffs[1] * CppAD::pow(x, 1)\
			  + coeffs[2] * CppAD::pow(x, 2) + coeffs[3] * CppAD::pow(x, 3);
		  AD<double> psi_fit = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * pow(x, 2));

		  //std::cout << "y_fit " << y_fit << std::endl;
		  // ��i����ľ���ƫ��
		  fg[0] = fg[0] + cte_cost_weight * CppAD::pow((y - y_fit), 2);
		  // ��i����ĽǶ�ƫ�� 
		  fg[0] = fg[0] + epsi_cost_weight * CppAD::pow((psi - psi_fit), 2);
		  // ��i���� ��Ŀ���ٶȵ�ƫ��
		  fg[0] = fg[0] + v_cost_weight * CppAD::pow((v - ref_v), 2);

		  // Ԥ��� i+1 ��λ��
		  if (i <= (N - 2))
		  {
			  AD<double> delta = vars[i * 6 + 4];   // ת��;
			  AD<double> a = vars[i * 6 + 5];       // ���ٶȣ����ţ�;

			  AD<double> next_x = x + v * CppAD::cos(psi) * dt;
			  AD<double> next_y = y + v * CppAD::sin(psi) * dt;
			  AD<double> next_psi = psi - v * delta/ Lf * dt;
			  AD<double> next_v = v + a * dt;

			  //std::cout << "next_y " << next_y << std::endl;
              //std::cout << "next_psi " << next_x << std::endl;
              //std::cout << "next_v " << next_v << std::endl;

			  // ��i���� �ļ��ٶ���ת��ǣ�Խ�ٵĲ���Խ�ã�
			  fg[0] = fg[0] + delta_cost_weight * CppAD::pow(delta, 2);
			  fg[0] = fg[0] + a_cost_weight * CppAD::pow(a, 2);

			  fg[i * 4 + 1] = next_x - vars[(i + 1) * 6];        // next_x �� ��������һ��x Ӧ��Ϊ0
			  fg[i * 4 + 2] = next_y - vars[(i + 1) * 6 + 1];    // next_y �� ��������һ��y Ӧ��Ϊ0
			  fg[i * 4 + 3] = next_psi - vars[(i + 1) * 6 + 2];  // next_psi �� ��������һ��psi Ӧ��Ϊ0
			  fg[i * 4 + 4] = next_v - vars[(i + 1) * 6 + 3];    // next_v �� ��������һ��v Ӧ��Ϊ0

			  // ��i����ļ��ٶ���ת��ǵı仯�ʣ�Խ�ٵı仯Խ�ã�
			  AD<double> last_delta = 0;
			  AD<double> last_a = 0;
			  if (i >= 1)
			  {
				  last_delta = vars[(i - 1) * 6 + 4];
				  last_a = vars[(i - 1) * 6 + 5];
				  fg[0] = fg[0] + delta_change_cost_weight * CppAD::pow((delta - last_delta), 2);
				  fg[0] = fg[0] + a_cost_weight * CppAD::pow((a - last_a), 2);
			  }
		  }
	  }
	  // ��ʼ״̬�����ڴ���������
	  fg[(N - 1) * 4 + 1] = vars[0];  // x 
	  fg[(N - 1) * 4 + 2] = vars[1];  // y
	  fg[(N - 1) * 4 + 3] = vars[2];  // psi
	  fg[(N - 1) * 4 + 4] = vars[3];  // v
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  /**
   * TODO: Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9 
   */
  size_t n_vars = 4 * N + 2 * (N - 1);
  /**
   * TODO: Set the number of constraints
   */
  size_t n_constraints = 4 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned  i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  /**
   * TODO: Set lower and upper limits for variables.
   */
  for (unsigned  i = 0; i <= N - 1; i++)
  {
	  // stateֵ��������, ��double���͵������Сֵ
	  for (unsigned  j = 0; j <= 3; j++)
	  {
		  vars_lowerbound[i * 6 + j] = -1.0e19;
		  vars_upperbound[i * 6 + j] = 1.0e19;
	  }
  }

  for (unsigned i = 0; i <= N - 2; i++)
  {
	  // ת�� ���� -25 and 25��
	  vars_lowerbound[i * 6 + 4] = -0.436332;
	  vars_upperbound[i * 6 + 4] = 0.436332;

	  // ���ٶ� ����ֵ -1��+1
	  vars_lowerbound[i * 6 + 5] = -1.0;
	  vars_upperbound[i * 6 + 5] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned  i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // ��ʼ״̬���ڲ���ֵ
  constraints_lowerbound[4 * (N - 1)] = x;
  constraints_upperbound[4 * (N - 1)] = x;

  constraints_lowerbound[4 * (N - 1) + 1] = y;
  constraints_upperbound[4 * (N - 1) + 1] = y;

  constraints_lowerbound[4 * (N - 1) + 2] = psi;
  constraints_upperbound[4 * (N - 1) + 2] = psi;

  constraints_lowerbound[4 * (N - 1) + 3] = v;
  constraints_upperbound[4 * (N - 1) + 3] = v;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

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
  std::cout << "Cost " << cost << std::endl;

  std::cout << "x " << solution.x[0] << std::endl;
  std::cout << "y " << solution.x[1] << std::endl;
  std::cout << "psi " << solution.x[2] << std::endl;
  std::cout << "v " << solution.x[3] << std::endl;
  std::cout << "steer_value " << solution.x[4] << std::endl;
  std::cout << "throttle_value " << solution.x[5] << std::endl;

  /**
   * TODO: Return the first actuator values. The variables can be accessed with
   *   `solution.x[i]`.
   *
   * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
   *   creates a 2 element double vector.
   */
  std::vector<double> result;	
  result.push_back(solution.x[4]);    // delta
  result.push_back(solution.x[5]);    // a
  for (int i = 0; i < N; i++) 
  {
	  result.push_back(solution.x[i * 6]);      // x
	  result.push_back(solution.x[i * 6 + 1]);  // y
  }

  return result;
}

