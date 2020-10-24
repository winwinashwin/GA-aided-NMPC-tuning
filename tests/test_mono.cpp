#include "pcheaders.h"

#include "genetic_algorithm/core.h"
#include "genetic_algorithm/operators.h"
#include "model/differential_drive.h"
#include "model/base_organism.h"
#include "utils/config_handler.hpp"
#include <gtest/gtest.h>

static size_t mpcTestLoop(void)
{
    mpc::Params params;

    model::DifferentialDrive dModel;
    model::TerminateOn<config::MONO> term;

    size_t count = 0;

    params.forward.timesteps = 12;
    params.forward.dt = 0.1;

    params.desired.vel = 0.5;
    params.desired.cte = 0.0;
    params.desired.etheta = 0.0;

    params.limits.omega = {-2.0, 2.0};
    params.limits.throttle = {-1.0, 1.0};

    params.weights.cte = 87.859183;
    params.weights.etheta = 99.532785;
    params.weights.vel = 54.116644;
    params.weights.omega = 47.430096;
    params.weights.acc = 2.185306;
    params.weights.omega_d = 4.611500;
    params.weights.acc_d = 66.870729;

    term.tolerance.vel = 0.01;
    term.tolerance.cte = 0.01;
    term.tolerance.etheta = 0.01;

    dModel.setSampleTime(params.forward.dt);
    dModel.setInitState(model::State({-8.0, 1.5, -0.6, 0.0, 0.0, 0.0}));

    bool done = false;

    while (!done)
    {
        std::array<double, 40> ptsx;
        std::array<double, 40> ptsy;

        const model::State &state = dModel.getState();

        for (size_t i = 0; i < 40; i++)
        {
            ptsx[i] = state.x + i * 0.1;
            ptsy[i] = 0.0;
        }

        const double &px = state.x;
        const double &py = state.y;
        const double &theta = state.theta;
        const double &v = state.linVel;
        double omega = state.angVel;
        double throttle = state.throttle;

        for (size_t i = 0; i < ptsx.size(); i++)
        {
            const double shift_x = ptsx[i] - px;
            const double shift_y = ptsy[i] - py;
            ptsx[i] = shift_x * cos(-theta) - shift_y * sin(-theta);
            ptsy[i] = shift_x * sin(-theta) + shift_y * cos(-theta);
        }

        double *ptrx = &ptsx[0];
        Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

        double *ptry = &ptsy[0];
        Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

        const Eigen::VectorXd &coeffs = mpc::utils::polyfit(ptsx_transform, ptsy_transform, 3);

        const double &cte = mpc::utils::polyeval(coeffs, 0);
        const double &etheta = -atan(coeffs[1]);

        if (abs(cte) > 10)
        {
            DEBUG_LOG("CTE out of bounds!! Got: " << cte);
        }

        const double &dt = params.forward.dt;
        const double current_px = 0.0 + v * dt;
        const double current_py = 0.0;
        const double current_theta = 0.0 + omega * dt;
        const double current_v = v + throttle * dt;
        const double current_cte = cte + v * sin(etheta) * dt;
        const double current_etheta = etheta - current_theta;

        Eigen::VectorXd model_state(6);
        model_state << current_px, current_py, current_theta, current_v, current_cte, current_etheta;

        // time to solve !
        mpc::MPC _mpc(params, coeffs);
        const std::vector<double> &mpc_solns = _mpc.solve(model_state);

        omega = mpc_solns[0];
        throttle = mpc_solns[1];

        const double speed = current_v + throttle * dt;

        dModel.step(speed, omega);

        count++;

        done = abs(current_cte) < term.tolerance.cte &&
               abs(current_v - params.desired.vel) < term.tolerance.vel &&
               abs(current_etheta) < term.tolerance.etheta;
    }

    return count;
}

TEST(GaMonoTestSuite, testBinMono)
{
    ASSERT_EQ(mpcTestLoop(), 493);
}