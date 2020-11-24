#include "model/base_organism.h"

namespace model
{
    template <>
    bool BaseOrganism<config::MONO>::followSetpoints(const mpc::Params &params, const TerminateOn<config::MONO> &term)
    {
        m_dModel.setSampleTime(params.forward.dt);

        double prevOmega = 0.0, prevSpeed = 0.0;
        size_t count = 0;

        try
        {
            bool done = false;

            while (!done)
            {
                std::array<double, 40> ptsx;
                std::array<double, 40> ptsy;

                const State state = m_dModel.getState();

                for (size_t i = 0; i < 40; i++)
                {
                    ptsx[i] = state.x + i * 0.1;
                    ptsy[i] = 0.0;
                }

                const double px = state.x;
                const double py = state.y;
                const double theta = state.theta;
                const double v = state.linVel;
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

                Eigen::VectorXd coeffs = mpc::utils::polyfit(ptsx_transform, ptsy_transform, 3);

                const double cte = mpc::utils::polyeval(coeffs, 0);
                const double etheta = -atan(coeffs[1]);

                if (abs(cte) > 10)
                    DEBUG_LOG("CTE out of bounds!! Got: " << cte);

                const double dt = params.forward.dt;
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

                double speed = current_v + throttle * dt;

                m_dModel.step(speed, omega);

                count++;
                CONSOLE_LOG(" [ INFO ]: Updating internal model ... timestep " << count << "\r");

                m_jsonLogger.logX(px);
                m_jsonLogger.logY(py);
                m_jsonLogger.logVelError(current_v - params.desired.vel);
                m_jsonLogger.logCte(current_cte);
                m_jsonLogger.logEtheta(current_etheta);
                m_jsonLogger.logCost(mpc_solns[2]);

                m_performance.cteData.push_back(current_cte);
                m_performance.ethetaData.push_back(current_etheta);
                m_performance.velErrData.push_back(params.desired.vel - current_v);

                m_performance.translationalEL.push_back(pow(speed, 2) - pow(prevSpeed, 2));
                m_performance.rotationalEL.push_back(pow(omega, 2) - pow(prevOmega, 2));

                prevSpeed = speed;
                prevOmega = omega;

                done = abs(current_cte) < term.tolerance.cte &&
                       abs(current_v - params.desired.vel) < term.tolerance.vel &&
                       abs(current_etheta) < term.tolerance.etheta;
            }

            CONSOLE_LOG("\n -- Run complete. Took " << count << " iterations" << std::endl);
        }
        catch (std::exception &e)
        {
            DEBUG_LOG("Error in NMPC control loop: " << e.what());
            return false;
        }

        return true;
    }

    template <>
    bool BaseOrganism<config::GA>::followSetpoints(const mpc::Params &params, const TerminateOn<config::GA> &term)
    {
        m_dModel.setSampleTime(params.forward.dt);

        double prevOmega = 0.0, prevSpeed = 0.0;

        try
        {
            for (size_t count = 0; count < term.iterations; count++)
            {
                std::array<double, 40> ptsx;
                std::array<double, 40> ptsy;

                const State state = m_dModel.getState();

                for (size_t i = 0; i < 40; i++)
                {
                    ptsx[i] = state.x + i * 0.1;
                    ptsy[i] = 0.0;
                }

                const double px = state.x;
                const double py = state.y;
                const double theta = state.theta;
                const double v = state.linVel;
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

                Eigen::VectorXd coeffs = mpc::utils::polyfit(ptsx_transform, ptsy_transform, 3);

                const double cte = mpc::utils::polyeval(coeffs, 0);
                const double etheta = -atan(coeffs[1]);

                if (abs(cte) > 10)
                    DEBUG_LOG("CTE out of bounds!! Got: " << cte);

                const double dt = params.forward.dt;
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
                const double cost = mpc_solns[2];

                const double speed = current_v + throttle * dt;

                m_dModel.step(speed, omega);
                const double velError = current_v - params.desired.vel;

                m_jsonLogger.logX(px);
                m_jsonLogger.logY(py);
                m_jsonLogger.logVelError(velError);
                m_jsonLogger.logCte(current_cte);
                m_jsonLogger.logEtheta(current_etheta);
                m_jsonLogger.logCost(cost);

                m_performance.cteData.push_back(current_cte);
                m_performance.ethetaData.push_back(current_etheta);
                m_performance.velErrData.push_back(velError);

                m_performance.translationalEL.push_back(pow(speed, 2) - pow(prevSpeed, 2));
                m_performance.rotationalEL.push_back(pow(omega, 2) - pow(prevOmega, 2));

                prevSpeed = speed;
                prevOmega = omega;
            }
        }

        catch (std::exception &e)
        {
            DEBUG_LOG("Error in NMPC control loop: " << e.what());
            return false;
        }

        return true;
    } // namespace model
} // namespace model
