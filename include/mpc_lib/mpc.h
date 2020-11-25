#ifndef DIFF_DRIVE_MPC_H_
#define DIFF_DRIVE_MPC_H_

#include "primary.h"
#include <eigen3/Eigen/Core>
#include <cppad/cppad.hpp>
/**
 * Utilities/helpers for NMPC
 */
namespace mpc::utils
{
    /**
     * Evaluate a polynomial
     * 
     * @param coeffs: Coefficients of the polynomial, constant term first
     * @param x: The X value at which the polynomial is to be evaluated
     * 
     * @return Result
     */
    double polyeval(const Eigen::VectorXd &coeffs, double x);

    /**
     * Find best fit polynomial coefficients
     * 
     * @param xvals: The x values
     * @param yvals: The y values
     * @param order: Order of the polynomial
     * 
     * @return The coefficients
     */
    Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);
} // namespace mpc::utils

namespace mpc
{
    /// Structure to store maximun and minimum allowed values
    template <typename __T>
    struct __LH
    {
        __T min, max;
    };

    struct Params
    {
        struct __Forward
        {
            /// Number of timesteps in the prediction horizon
            size_t timesteps;
            /// Sample time of the controller
            double dt;
        } forward;

        /// Desired errors and velocity, plant will try to achieve these / stay close to these values
        struct __Desired
        {
            double cte;
            double etheta;
            double vel;
        } desired;

        /// This stores the constraints
        struct __Limits
        {
            __LH<double> omega, throttle;
        } limits;

        /// This will be used as the default constraint
        const double BOUND_VALUE;

        /// Weights for the cost function
        struct Weights
        {
            double vel, cte, etheta, omega, acc, omega_d, acc_d;

            operator std::string() const;
        } weights;

        Params();
    };

    /// Helper struct to store indices of variables
    struct VarIndices
    {
        size_t x_start, y_start, theta_start;
        size_t v_start, omega_start, acc_start;
        size_t cte_start, etheta_start;

        /**
         * Constructor
         * 
         * Initialises all indicies based on the timesteps
         */
        explicit VarIndices(size_t timesteps);
    };

    /// Main class for MPC implementation
    class MPC
    {
    public:
        typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

        /**
         * Constructor
         * 
         * @param params: The parameters for the MPC
         * @param coeffs: The coefficients of the best fit polynomial
         */
        MPC(const Params &params, const Eigen::VectorXd &coeffs);

        /// Destructor
        ~MPC();

        void operator()(ADvector &fg, const ADvector &vars) const;

        /**
         * Solve the NLP
         * 
         * @param state: Current state of the model
         * 
         * @return Vector of manipulated variables and other parameters like cost
         */
        std::vector<double> solve(Eigen::VectorXd &state);

    private:
        const Params m_Params;
        const Eigen::VectorXd m_Coeffs;
        const VarIndices m_VarIndices;
    };
} // namespace mpc

#endif // DIFF_DRIVE_MPC_H_