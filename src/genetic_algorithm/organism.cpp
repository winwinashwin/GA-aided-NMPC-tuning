#include "genetic_algorithm/organism.h"
#include "utils/config_handler.hpp"

namespace ga
{
    Organism::Organism()
    {
        const auto mpcConfig = config::ConfigHandler<config::GA>::getMpcConfig();

        m_genome.addChoromosome(mpcConfig.weight_bounds.w_vel.first, mpcConfig.weight_bounds.w_vel.second);
        m_genome.addChoromosome(mpcConfig.weight_bounds.w_cte.first, mpcConfig.weight_bounds.w_cte.second);
        m_genome.addChoromosome(mpcConfig.weight_bounds.w_etheta.first, mpcConfig.weight_bounds.w_etheta.second);
        m_genome.addChoromosome(mpcConfig.weight_bounds.w_omega.first, mpcConfig.weight_bounds.w_omega.second);
        m_genome.addChoromosome(mpcConfig.weight_bounds.w_acc.first, mpcConfig.weight_bounds.w_acc.second);
        m_genome.addChoromosome(mpcConfig.weight_bounds.w_omega_d.first, mpcConfig.weight_bounds.w_omega_d.second);
        m_genome.addChoromosome(mpcConfig.weight_bounds.w_acc_d.first, mpcConfig.weight_bounds.w_acc_d.second);

        model::State s = {
            mpcConfig.initial_state.x,
            mpcConfig.initial_state.y,
            mpcConfig.initial_state.theta,
            mpcConfig.initial_state.linear_velocity,
            mpcConfig.initial_state.angular_velocity,
            mpcConfig.initial_state.throttle};

        setModelInitState(s);
    }

    double Organism::getFitness() const
    {
        return m_fitness;
    }

    mpc::Params::Weights Organism::getWeights() const
    {
        return m_genome.decode();
    }

    ga::core::Genome Organism::getGenome() const
    {
        return m_genome;
    }

    void Organism::setFitness(double fitness)
    {
        m_fitness = fitness;
    }

    void Organism::setWeights(const mpc::Params::Weights &weights)
    {
        m_genome.encode(weights);
    }

    void Organism::setGenome(const ga::core::Genome &genome)
    {
        m_genome = genome;
    }

    void Organism::saveAsBest(size_t genCount)
    {
        const mpc::Params::Weights &w = getWeights();

        m_jsonLogger.logWeights(
            w.vel,
            w.cte,
            w.etheta,
            w.omega,
            w.acc,
            w.omega_d,
            w.acc_d);

        std::string name = "data/best-of-generation-" + std::to_string(genCount) + ".json";

        m_jsonLogger.dump(name);
    }

} // namespace ga