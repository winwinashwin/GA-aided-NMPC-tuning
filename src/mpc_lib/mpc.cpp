#include "mpc_lib/mpc.h"
#include <Eigen/QR>
#include <cppad/ipopt/solve.hpp>

namespace mpc::utils
{
    double polyeval(const Eigen::VectorXd &coeffs, double x)
    {
        double result = 0.0;

        for (int i = 0; i < coeffs.size(); i++)
            result += coeffs[i] * pow(x, i);

        return result;
    }

    Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order)
    {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
            A(i, 0) = 1.0;

        for (int j = 0; j < xvals.size(); j++)
            for (int i = 0; i < order; i++)
                A(j, i + 1) = A(j, i) * xvals(j);

        Eigen::VectorXd result = A.householderQr().solve(yvals);

        return result;
    }
} // namespace mpc::utils

namespace mpc
{
    Params::Weights::operator std::string() const
    {
        using namespace std;

        string result(" -- \n");
        result += "\tw_vel:      " + to_string(vel) + "\n";
        result += "\tw_cte:      " + to_string(cte) + "\n";
        result += "\tw_etheta:   " + to_string(etheta) + "\n";
        result += "\tw_omega:    " + to_string(omega) + "\n";
        result += "\tw_acc:      " + to_string(acc) + "\n";
        result += "\tw_omega_d:  " + to_string(omega_d) + "\n";
        result += "\tw_acc_d:    " + to_string(acc_d) + "\n";
        result += " -- ";

        return result;
    }

    Params::Params() : BOUND_VALUE(1.0e3)
    {
    }

    VarIndices::VarIndices(size_t timesteps)
    {
          x_start = 0;
          y_start = x_start + timesteps;
          theta_start = y_start + timesteps;
          v_start = theta_start + timesteps;
          cte_start = v_start + timesteps;
          etheta_start = cte_start + timesteps;
          omega_start = etheta_start + timesteps;
          acc_start = omega_start + timesteps - 1;
    }

    MPC::MPC(const Params &params, const Eigen::VectorXd &coeffs) : m_Params(params),
                                                                    m_Coeffs(coeffs),
                                                                    m_VarIndices(params.forward.timesteps)
    {
    }

    MPC::~MPC()
    {
    }

    void MPC::operator()(ADvector &fg, const ADvector &vars) const
    {
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // Reference State Cost
        for (size_t t = 0; t < m_Params.forward.timesteps; t++)
        {
            fg[0] += m_Params.weights.cte * CppAD::pow(vars[m_VarIndices.cte_start + t] - m_Params.desired.cte, 2);
            fg[0] += m_Params.weights.etheta * CppAD::pow(vars[m_VarIndices.etheta_start + t] - m_Params.desired.etheta, 2);
            fg[0] += m_Params.weights.vel * CppAD::pow(vars[m_VarIndices.v_start + t] - m_Params.desired.vel, 2);
        }
        for (size_t t = 0; t < m_Params.forward.timesteps - 1; t++)
        {
            fg[0] += m_Params.weights.omega * CppAD::pow(vars[m_VarIndices.omega_start + t], 2);
            fg[0] += m_Params.weights.acc * CppAD::pow(vars[m_VarIndices.acc_start + t], 2);
        }
        // Smoother transitions (less jerks)
        for (size_t t = 0; t < m_Params.forward.timesteps - 2; t++)
        {
            fg[0] += m_Params.weights.acc_d * CppAD::pow(vars[m_VarIndices.acc_start + t + 1] - vars[m_VarIndices.acc_start + t], 2);
            fg[0] += m_Params.weights.omega_d * CppAD::pow(vars[m_VarIndices.omega_start + t + 1] - vars[m_VarIndices.omega_start + t], 2);
        }
        //
        // Setup Constraints
        //

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + m_VarIndices.x_start] = vars[m_VarIndices.x_start];
        fg[1 + m_VarIndices.y_start] = vars[m_VarIndices.y_start];
        fg[1 + m_VarIndices.theta_start] = vars[m_VarIndices.theta_start];
        fg[1 + m_VarIndices.v_start] = vars[m_VarIndices.v_start];
        fg[1 + m_VarIndices.cte_start] = vars[m_VarIndices.cte_start];
        fg[1 + m_VarIndices.etheta_start] = vars[m_VarIndices.etheta_start];

        // The rest of the constraints
        for (size_t t = 0; t < m_Params.forward.timesteps - 1; t++)
        {

            // Time : T + 1
            CppAD::AD<double> x1 = vars[m_VarIndices.x_start + t + 1];
            CppAD::AD<double> y1 = vars[m_VarIndices.y_start + t + 1];
            CppAD::AD<double> theta1 = vars[m_VarIndices.theta_start + t + 1];
            CppAD::AD<double> v1 = vars[m_VarIndices.v_start + t + 1];
            CppAD::AD<double> cte1 = vars[m_VarIndices.cte_start + t + 1];
            CppAD::AD<double> etheta1 = vars[m_VarIndices.etheta_start + t + 1];

            // Time : T
            CppAD::AD<double> x0 = vars[m_VarIndices.x_start + t];
            CppAD::AD<double> y0 = vars[m_VarIndices.y_start + t];
            CppAD::AD<double> theta0 = vars[m_VarIndices.theta_start + t];
            CppAD::AD<double> v0 = vars[m_VarIndices.v_start + t];
            CppAD::AD<double> cte0 = vars[m_VarIndices.cte_start + t];
            CppAD::AD<double> etheta0 = vars[m_VarIndices.etheta_start + t];

            CppAD::AD<double> w0 = vars[m_VarIndices.omega_start + t];
            CppAD::AD<double> a0 = vars[m_VarIndices.acc_start + t];

            CppAD::AD<double> f0 = 0.0;
            // CppAD::pow() takes second parameter as const int&, putting counter datatype as size_t throws error
            for (int i = 0; i < m_Coeffs.size(); i++)
                f0 += m_Coeffs[i] * CppAD::pow(x0, i);

            CppAD::AD<double> traj_grad0 = 0.0;
            for (int i = 1; i < m_Coeffs.size(); i++)
                traj_grad0 += i * m_Coeffs[i] * CppAD::pow(x0, i - 1);

            traj_grad0 = CppAD::atan(traj_grad0);

            // The idea here is to constraint this value to be 0.
            //
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            fg[2 + m_VarIndices.x_start + t] = x1 - (x0 + v0 * CppAD::cos(theta0) * m_Params.forward.dt);
            fg[2 + m_VarIndices.y_start + t] = y1 - (y0 + v0 * CppAD::sin(theta0) * m_Params.forward.dt);
            fg[2 + m_VarIndices.theta_start + t] = theta1 - (theta0 + w0 * m_Params.forward.dt);
            fg[2 + m_VarIndices.v_start + t] = v1 - (v0 + a0 * m_Params.forward.dt);

            fg[2 + m_VarIndices.cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(etheta0) * m_Params.forward.dt));
            fg[2 + m_VarIndices.etheta_start + t] = etheta1 - ((theta0 - traj_grad0) + w0 * m_Params.forward.dt);
        }
    }

    std::vector<double> MPC::solve(Eigen::VectorXd &state)
    {
        bool ok = true;
        typedef CppAD::vector<double> Dvector;

        const double x = state[0];
        const double y = state[1];
        const double theta = state[2];
        const double v = state[3];
        const double cte = state[4];
        const double etheta = state[5];

        const size_t n_vars = 6 * m_Params.forward.timesteps + 2 * (m_Params.forward.timesteps - 1);
        const size_t n_constraints = 6 * m_Params.forward.timesteps;

        // Initial value of the independent variables.
        // SHOULD BE 0 besides initial state.
        Dvector vars(n_vars);
        for (size_t i = 0; i < n_vars; i++)
            vars[i] = 0;

        vars[m_VarIndices.x_start] = x;
        vars[m_VarIndices.y_start] = y;
        vars[m_VarIndices.theta_start] = theta;
        vars[m_VarIndices.v_start] = v;
        vars[m_VarIndices.cte_start] = cte;
        vars[m_VarIndices.etheta_start] = etheta;

        Dvector vars_lowerbound(n_vars);
        Dvector vars_upperbound(n_vars);

        for (size_t i = 0; i < m_VarIndices.omega_start; i++)
        {
            vars_lowerbound[i] = -m_Params.BOUND_VALUE;
            vars_upperbound[i] = m_Params.BOUND_VALUE;
        }
        for (size_t i = m_VarIndices.omega_start; i < m_VarIndices.acc_start; i++)
        {
            vars_lowerbound[i] = m_Params.limits.omega.min;
            vars_upperbound[i] = m_Params.limits.omega.max;
        }
        for (size_t i = m_VarIndices.acc_start; i < n_vars; i++)
        {
            vars_lowerbound[i] = m_Params.limits.throttle.min;
            vars_upperbound[i] = m_Params.limits.throttle.max;
        }

        Dvector constraints_lowerbound(n_constraints);
        Dvector constraints_upperbound(n_constraints);

        for (size_t i = 0; i < n_constraints; i++)
        {
            constraints_lowerbound[i] = 0;
            constraints_upperbound[i] = 0;
        }
        constraints_lowerbound[m_VarIndices.x_start] = x;
        constraints_lowerbound[m_VarIndices.y_start] = y;
        constraints_lowerbound[m_VarIndices.theta_start] = theta;
        constraints_lowerbound[m_VarIndices.v_start] = v;
        constraints_lowerbound[m_VarIndices.cte_start] = cte;
        constraints_lowerbound[m_VarIndices.etheta_start] = etheta;

        constraints_upperbound[m_VarIndices.x_start] = x;
        constraints_upperbound[m_VarIndices.y_start] = y;
        constraints_upperbound[m_VarIndices.theta_start] = theta;
        constraints_upperbound[m_VarIndices.v_start] = v;
        constraints_upperbound[m_VarIndices.cte_start] = cte;
        constraints_upperbound[m_VarIndices.etheta_start] = etheta;

        std::string options;

        // Uncomment this if you'd like more print information
        options += "Integer print_level  0\n";
        // NOTE: Setting sparse to true allows the solver to take advantage
        // of sparse routines, this makes the computation MUCH FASTER. If you
        // can uncomment 1 of these and see if it makes a difference or not but
        // if you uncomment both the computation time should go up in orders of
        // magnitude.
        options += "String  sb          yes\n"; // Disables printing IPOPT creator banner
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";
        // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
        // Change this as you see fit.
        options += "Numeric max_cpu_time          0.5\n";

        // place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;

        CppAD::ipopt::solve(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, *this, solution);

        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

        if (!ok)
            DEBUG_LOG("IPOPT returned unsuccessful solve. Code: " << static_cast<size_t>(solution.status));

        // Cost
        // std::cout << "COST  : " << solution.obj_value << std::endl;

        // Return the first actuator values. The variables can be accessed with
        // `solution.x[i]`.
        std::vector<double> result;
        result.reserve(3);

        result.push_back(solution.x[m_VarIndices.omega_start]);
        result.push_back(solution.x[m_VarIndices.acc_start]);
        result.push_back(solution.obj_value);

        // // Add "future" solutions (where MPC is going)
        // for (int i = 0; i < m_Params.forward.timesteps - 1; ++i)
        // {
        //     result.push_back(solution.x[m_VarIndices.x_start + i + 1]);
        //     result.push_back(solution.x[m_VarIndices.y_start + i + 1]);
        // }

        return result;
    }
} // namespace mpc