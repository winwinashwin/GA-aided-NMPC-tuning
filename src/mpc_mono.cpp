#include "pcheaders.h"

#include "model/base_organism.h"

class Organism : public model::BaseOrganism<config::MONO>
{
public:
    Organism()
    {
        m_jsonLogger = JsonLogger();

        const auto &config = config::ConfigHandler<config::MONO>::getMpcConfig();

        m_params.forward.timesteps = config.general.timesteps;
        m_params.forward.dt = config.general.sample_time;
        m_params.desired.vel = config.desired.velocity;
        m_params.desired.cte = config.desired.cross_track_error;
        m_params.desired.etheta = config.desired.orientation_error;
        m_params.limits.omega = {-config.max_bounds.omega, config.max_bounds.omega};
        m_params.limits.throttle = {-config.max_bounds.throttle, config.max_bounds.throttle};
        m_params.weights.cte = config.weights.w_cte;
        m_params.weights.etheta = config.weights.w_etheta;
        m_params.weights.vel = config.weights.w_vel;
        m_params.weights.omega = config.weights.w_omega;
        m_params.weights.acc = config.weights.w_acc;
        m_params.weights.omega_d = config.weights.w_omega_d;
        m_params.weights.acc_d = config.weights.w_acc_d;

        m_jsonLogger.logWeights(
            m_params.weights.vel,
            m_params.weights.cte,
            m_params.weights.etheta,
            m_params.weights.omega,
            m_params.weights.acc,
            m_params.weights.omega_d,
            m_params.weights.acc_d);

        m_term.tolerance.vel = config.teardown_tolerance.velocity;
        m_term.tolerance.cte = config.teardown_tolerance.cross_track_error;
        m_term.tolerance.etheta = config.teardown_tolerance.orientation_error;
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
        {
            DEBUG_LOG("Control loop fail !");
        }
    }

private:
    mpc::Params m_params;
    model::TerminateOn<config::MONO> m_term;
};

int main(int argc, char **argv)
{
    DEBUG_LOG("Binary built in debug mode. If not intended, abort.");

    const model::State s = {-8.0, 1.5, -0.6, 0.0, 0.0, 0.0};

    Organism *organism = new Organism();

    organism->setModelInitState(s);

    organism->run();

    organism->saveData();

    delete organism;
}