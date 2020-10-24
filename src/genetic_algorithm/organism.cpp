#include "genetic_algorithm/organism.h"
#include "utils/config_handler.hpp"

namespace ga
{
    Organism::Organism()
    {
        const auto &config = config::ConfigHandler<config::GA>::getMpcConfig();

        m_genome.addChoromosome(config.weight_bounds.w_vel.first, config.weight_bounds.w_vel.second);
        m_genome.addChoromosome(config.weight_bounds.w_cte.first, config.weight_bounds.w_cte.second);
        m_genome.addChoromosome(config.weight_bounds.w_etheta.first, config.weight_bounds.w_etheta.second);
        m_genome.addChoromosome(config.weight_bounds.w_omega.first, config.weight_bounds.w_omega.second);
        m_genome.addChoromosome(config.weight_bounds.w_acc.first, config.weight_bounds.w_acc.second);
        m_genome.addChoromosome(config.weight_bounds.w_omega_d.first, config.weight_bounds.w_omega_d.second);
        m_genome.addChoromosome(config.weight_bounds.w_acc_d.first, config.weight_bounds.w_acc_d.second);

        model::State s = {-8.0, 1.5, -0.6, 0.0, 0.0, 0.0};

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

    void Organism::setFitness(const double &fitness)
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

    void Organism::saveAsBest(const size_t &genCount)
    {
        const mpc::Params::Weights &w = getWeights();

        m_jsonLogger.logWeights(
            w.vel,
            w.cte,
            w.etheta,
            w.omega,
            w.acc,
            w.omega_d,
            w.acc_d
        );

        std::string name = "data/best-of-generation-" + std::to_string(genCount) + ".json";

        m_jsonLogger.dump(name);
    }

} // namespace ga