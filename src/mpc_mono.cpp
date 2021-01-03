#include "model/base_organism.h"

static const auto mpcConfig = config::ConfigHandler<config::MONO>::getMpcConfig();

class Organism : public model::BaseOrganism<config::MONO>
{
public:
    Organism()
    {
        m_jsonLogger = JsonLogger();

        m_params.forward.timesteps = mpcConfig.general.timesteps;
        m_params.forward.dt = mpcConfig.general.sample_time;
        m_params.desired.vel = mpcConfig.desired.velocity;
        m_params.desired.cte = mpcConfig.desired.cross_track_error;
        m_params.desired.etheta = mpcConfig.desired.orientation_error;
        m_params.limits.omega = {-mpcConfig.max_bounds.omega, mpcConfig.max_bounds.omega};
        m_params.limits.throttle = {-mpcConfig.max_bounds.throttle, mpcConfig.max_bounds.throttle};
        m_params.weights.cte = mpcConfig.weights.w_cte;
        m_params.weights.etheta = mpcConfig.weights.w_etheta;
        m_params.weights.vel = mpcConfig.weights.w_vel;
        m_params.weights.omega = mpcConfig.weights.w_omega;
        m_params.weights.acc = mpcConfig.weights.w_acc;
        m_params.weights.omega_d = mpcConfig.weights.w_omega_d;
        m_params.weights.acc_d = mpcConfig.weights.w_acc_d;

        m_jsonLogger.logWeights(
            m_params.weights.vel,
            m_params.weights.cte,
            m_params.weights.etheta,
            m_params.weights.omega,
            m_params.weights.acc,
            m_params.weights.omega_d,
            m_params.weights.acc_d);

        m_term.tolerance.vel = mpcConfig.teardown_tolerance.velocity;
        m_term.tolerance.cte = mpcConfig.teardown_tolerance.cross_track_error;
        m_term.tolerance.etheta = mpcConfig.teardown_tolerance.orientation_error;
    }

    void saveData()
    {
        std::string destn = "data/custom-weights.json";

        m_jsonLogger.dump(destn);
        CONSOLE_LOG(" -- Data saved to " << destn << "\n\n");
    }

    void run()
    {
        const bool &ok = followSetpoints(m_params, m_term);

        if (!ok)
            DEBUG_LOG("Control loop fail !");
    }

private:
    mpc::Params m_params;
    model::TerminateOn<config::MONO> m_term;
};

int main(int argc, char **argv)
{
    DEBUG_LOG("Binary built in debug mode. If not intended, abort.");

    model::State s;
    s.x = mpcConfig.initial_state.x;
    s.y = mpcConfig.initial_state.y;
    s.theta = mpcConfig.initial_state.theta;
    s.linVel = mpcConfig.initial_state.linear_velocity;
    s.angVel = mpcConfig.initial_state.angular_velocity;
    s.throttle = mpcConfig.initial_state.throttle;

    Organism *organism = new Organism();

    organism->setModelInitState(s);
    organism->run();
    organism->saveData();

    delete organism;
}