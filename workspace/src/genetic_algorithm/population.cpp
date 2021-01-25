#include "genetic_algorithm/population.h"
#include "genetic_algorithm/operators.h"
#include "genetic_algorithm/fitness.h"
#include "utils/config_handler.hpp"
#include <random>

static bool sortByFitness(const ga::Organism &a, const ga::Organism &b)
{
    return (a.getFitness() > b.getFitness());
}

namespace ga
{
    static const auto gaConfig = config::ConfigHandler<config::GA>::getGAConfig();
    static const auto mpcConfig = config::ConfigHandler<config::GA>::getMpcConfig();

    Population::Population(size_t size, size_t matingPoolSize)
        : m_popSize(size),
          m_matingPoolSize(matingPoolSize)
    {
        m_organisms.reserve(size);

        for (size_t i = 0; i < size; i++)
            m_organisms.emplace_back();
    }

    void Population::randDistInit()
    {
        typedef std::uniform_real_distribution<double> Rd_t;
        // Generator for the distribution
        std::random_device rand_dev;
        std::mt19937 generator(rand_dev());

        // Distributions for different weights
        Rd_t dist_vel(mpcConfig.weight_bounds.w_vel.first, mpcConfig.weight_bounds.w_vel.second);
        Rd_t dist_cte(mpcConfig.weight_bounds.w_cte.first, mpcConfig.weight_bounds.w_cte.second);
        Rd_t dist_etheta(mpcConfig.weight_bounds.w_etheta.first, mpcConfig.weight_bounds.w_etheta.second);
        Rd_t dist_omega(mpcConfig.weight_bounds.w_omega.first, mpcConfig.weight_bounds.w_omega.second);
        Rd_t dist_acc(mpcConfig.weight_bounds.w_acc.first, mpcConfig.weight_bounds.w_acc.second);
        Rd_t dist_omega_d(mpcConfig.weight_bounds.w_omega_d.first, mpcConfig.weight_bounds.w_omega_d.second);
        Rd_t dist_acc_d(mpcConfig.weight_bounds.w_acc_d.first, mpcConfig.weight_bounds.w_acc_d.second);

        for (size_t i = 0; i < m_popSize; i++)
        {
            mpc::Params::Weights w;

            w.vel = dist_vel(generator);
            w.cte = dist_cte(generator);
            w.etheta = dist_etheta(generator);
            w.omega = dist_omega(generator);
            w.acc = dist_acc(generator);
            w.omega_d = dist_omega_d(generator);
            w.acc_d = dist_acc_d(generator);

            m_organisms[i].setWeights(w);
        }
    }

    void Population::mainLoop()
    {
        _updateFitnessVals();
        std::sort(m_organisms.begin(), m_organisms.end(), sortByFitness);

        _crossover();
        _mutation();
    }

    double Population::getBestFitness() const
    {
        return m_organisms[0].getFitness();
    }

    std::string Population::getBestWeights() const
    {
        return static_cast<std::string>(m_organisms[0].getWeights());
    }

    void Population::runIDT() const
    {
        ga::fitness::ObjFunction::interactiveDCT(m_organisms[0].getPerformance());
    }

    void Population::refresh(size_t gen_count)
    {
        m_organisms[0].saveAsBest(gen_count);

        for (size_t i = 0; i < m_popSize; i++)
            m_organisms[i].refresh();
    }

    void Population::_updateFitnessVals()
    {
        mpc::Params params;

        params.forward.timesteps = mpcConfig.general.timesteps;
        params.forward.dt = mpcConfig.general.sample_time;
        params.desired.vel = mpcConfig.desired.velocity;
        params.desired.cte = mpcConfig.desired.cross_track_error;
        params.desired.etheta = mpcConfig.desired.orientation_error;
        params.limits.omega = {-mpcConfig.max_bounds.omega, mpcConfig.max_bounds.omega};
        params.limits.throttle = {-mpcConfig.max_bounds.throttle, mpcConfig.max_bounds.throttle};

        model::TerminateOn<config::GA> condn;
        condn.iterations = gaConfig.general.iterations_per_genome;

        for (size_t i = 0; i < m_popSize; i++)
        {
            params.weights = m_organisms[i].getWeights();
            const bool ok = m_organisms[i].followSetpoints(params, condn);

            if (!ok)
                DEBUG_LOG("Control loop fail!");

            const double fitness = ga::fitness::ObjFunction::evaluate(m_organisms[i].getPerformance());

            m_organisms[i].setFitness(fitness);

            CONSOLE_LOG(" " << i + 1 << "/" << m_popSize << " ");
            m_pBar.show(static_cast<double>(i + 1) * 100 / m_popSize);
        }

        m_pBar.done();
    }

    void Population::_crossover()
    {
        /**
         * We keep the parents in the new population along with the progenies. This is done because if all progenies
         * turn out to be less fit than their parents, we can carry on the same parents in the next crossover
         */

        ga::core::Genome parent1, parent2;

        for (size_t k = m_matingPoolSize; k < m_popSize; k++)
        {
            parent1 = m_organisms[k % m_matingPoolSize].getGenome();
            parent2 = m_organisms[(k + 1) % m_matingPoolSize].getGenome();

            ga::core::Genome childGenome = ga::operators::crossover::uniform(parent1, parent2);

            m_organisms[k].setGenome(childGenome);
        }
    }

    void Population::_mutation()
    {
        const double probab = config::ConfigHandler<config::GA>::getGAConfig().operators.mutation_probability;

        for (size_t k = m_matingPoolSize; k < m_popSize; k++)
            m_organisms[k].setGenome(ga::operators::mutation::bitFlip(m_organisms[k].getGenome(), probab));
    }

} // namespace ga