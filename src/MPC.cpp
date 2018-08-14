#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_v = 30;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
        // cost in the first element of fg
        fg[0] = 0.0;

        // loop over states to add up cost
        for (int step = 0; step < N; step++) {
            fg[0] += 400 * CppAD::pow(vars[cte_start + step], 4);
            fg[0] += 4000 * CppAD::pow(vars[epsi_start + step], 4);
            fg[0] += 1.0 * CppAD::pow(vars[v_start + step] - ref_v, 2);
        }

        // minimize use of controls
        for (int step = 0; step < N - 1; step++) {
            fg[0] += 300 * CppAD::pow(vars[delta_start + step], 2);
            fg[0] += 20 * CppAD::pow(vars[a_start + step], 2);

            // add cost for closing the distance too quickly
            fg[0] += 1000 * CppAD::pow(vars[cte_start + step + 1] - vars[cte_start + step], 2);
        }

        // minimize change in controls
        for (int step = 0; step < N - 2; step++) {
            fg[0] += 600 * CppAD::pow(vars[delta_start + step + 1] - vars[delta_start + step], 4);
            fg[0] += 600 * CppAD::pow(vars[a_start     + step + 1] - vars[a_start     + step], 2);
        }

        // setup initial constrains
        fg[1 + x_start]     = vars[x_start];
        fg[1 + y_start]     = vars[y_start];
        fg[1 + psi_start]   = vars[psi_start];
        fg[1 + v_start]     = vars[v_start];
        fg[1 + cte_start]   = vars[cte_start];
        fg[1 + epsi_start]  = vars[epsi_start];

        // set up motion model
        for (int step = 1; step < N; step++) {
            AD<double> x_cur     = vars[x_start + step];
            AD<double> y_cur     = vars[y_start + step];
            AD<double> psi_cur   = vars[psi_start + step];
            AD<double> v_cur     = vars[v_start + step];
            AD<double> cte_cur   = vars[cte_start + step];
            AD<double> epsi_cur  = vars[epsi_start + step];
            // actuators not needed for current calculation

            AD<double> x_pre     = vars[x_start + step - 1];
            AD<double> y_pre     = vars[y_start + step - 1];
            AD<double> psi_pre   = vars[psi_start + step - 1];
            AD<double> v_pre     = vars[v_start + step - 1];
            AD<double> cte_pre   = vars[cte_start + step - 1];
            AD<double> epsi_pre  = vars[epsi_start + step - 1];
            // do constrain actuators
            AD<double> delta_pre = vars[delta_start + step - 1];
            AD<double> a_pre     = vars[a_start + step - 1];

            // other
            AD<double> f_pre     = coeffs[0] + coeffs[1] * x_pre + coeffs[2] * CppAD::pow(x_pre, 2)
                                 + coeffs[3] * CppAD::pow(x_pre, 3);
            AD<double> psi_des_pre = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x_pre +
                                          3 * coeffs[3] * CppAD::pow(x_pre, 2));

            // now the constraints
            // x, y, psi, v
            fg[1 + x_start + step]    = x_cur -
                                        (x_pre + v_pre * dt * CppAD::cos(psi_pre));
            fg[1 + y_start + step]    = y_cur -
                                        (y_pre + v_pre * dt * CppAD::sin(psi_pre));

            // check this
            fg[1 + psi_start + step]  = psi_cur -
                                        (psi_pre - v_pre * dt * delta_pre / Lf);
            // what's going on here?

            fg[1 + v_start + step]    = v_cur -
                                        (v_pre + a_pre  * dt);
            fg[1 + cte_start + step]  = cte_cur -
                                        ((f_pre - y_pre) + v_pre * dt * CppAD::sin(epsi_pre));
            fg[1 + epsi_start + step] = epsi_cur -
                                        ((psi_pre - psi_des_pre) - v_pre * delta_pre * dt / Lf);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    size_t n_vars = 6 * N + 2 * (N - 1);
    size_t n_constraints = 6 * N;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    double x    = state[0];
    double y    = state[1];
    double psi  = state[2];
    double v    = state[3];
    double cte  = state[4];
    double epsi = state[5];

    // TODO: Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] =  1.0e19;
    }

    // Steering limits, in radians
    for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] =  0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -0.5; // gives the vehicle more power to avoid going off the road
        vars_upperbound[i] =  0.3; // higher than this is to rambunctious
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    // options += "Integer print_level  12\n";
    // // NOTE: Setting sparse to true allows the solver to take advantage
    // // of sparse routines, this makes the computation MUCH FASTER. If you
    // // can uncomment 1 of these and see if it makes a difference or not but
    // // if you uncomment both the computation time should go up in orders of
    // // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // // Change this as you see fit.
    // options += "Numeric max_cpu_time          0.5\n";

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

    // Return the first actuator values.
    vector<double> res;
    res.push_back(solution.x[delta_start]);
    res.push_back(solution.x[a_start]);

    for (int step = 0; step < N; step++) {
        res.push_back(solution.x[x_start + step]);
        res.push_back(solution.x[y_start + step]);
    }

    return res;
}
