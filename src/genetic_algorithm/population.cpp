#include "genetic_algorithm/population.h"
#include "genetic_algorithm/operators.h"
#include "genetic_algorithm/fitness.h"

static bool sortByFitness(const ga::Organism &a, const ga::Organism &b)
{
    return (a.getFitness() > b.getFitness());
}

namespace ga
{
    Population::Population(const size_t &size, const size_t &matingPoolSize)
        : m_popSize(size),
          m_matingPoolSize(matingPoolSize)
    {
        m_organisms.reserve(size);

        for (size_t i = 0; i < size; i++)
        {
            // Create obj directly in heap (push_back will create in stack and copy to heap)
            m_organisms.emplace_back(ga::Organism());
        }
    }

    void Population::randDistInit()
    {
        const auto &config = config::ConfigHandler<config::GA>::getMpcConfig();

        // Generator for the distribution
        std::random_device rand_dev;
        std::mt19937 generator(rand_dev());

        // Distributions for different weights
        std::uniform_real_distribution<double> dist_vel(config.weight_bounds.w_vel.first, config.weight_bounds.w_vel.second);
        std::uniform_real_distribution<double> dist_cte(config.weight_bounds.w_cte.first, config.weight_bounds.w_cte.second);
        std::uniform_real_distribution<double> dist_etheta(config.weight_bounds.w_etheta.first, config.weight_bounds.w_etheta.second);
        std::uniform_real_distribution<double> dist_omega(config.weight_bounds.w_omega.first, config.weight_bounds.w_omega.second);
        std::uniform_real_distribution<double> dist_acc(config.weight_bounds.w_acc.first, config.weight_bounds.w_acc.second);
        std::uniform_real_distribution<double> dist_omega_d(config.weight_bounds.w_omega_d.first, config.weight_bounds.w_omega_d.second);
        std::uniform_real_distribution<double> dist_acc_d(config.weight_bounds.w_acc_d.first, config.weight_bounds.w_acc_d.second);

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

    void Population::refresh(const size_t &gen_count)
    {
        m_organisms[0].saveAsBest(gen_count);

        for (size_t i = 0; i < m_popSize; i++)
        {
            m_organisms[i].refresh();
        }
    }

    void Population::_updateFitnessVals()
    {
        mpc::Params params;

        auto config = config::ConfigHandler<config::GA>::getMpcConfig();

        params.forward.timesteps = config.general.timesteps;
        params.forward.dt = config.general.sample_time;
        params.desired.vel = config.desired.velocity;
        params.desired.cte = config.desired.cross_track_error;
        params.desired.etheta = config.desired.orientation_error;
        params.limits.omega = {-config.max_bounds.omega, config.max_bounds.omega};
        params.limits.throttle = {-config.max_bounds.throttle, config.max_bounds.throttle};

        model::TerminateOn<config::GA> condn;
        condn.iterations = config::ConfigHandler<config::GA>::getGAConfig().general.iterations_per_genome;

        for (size_t i = 0; i < m_popSize; i++)
        {
            params.weights = m_organisms[i].getWeights();
            const bool &ok = m_organisms[i].followSetpoints(params, condn);

            if (!ok)
            {
                DEBUG_LOG("Control loop fail!");
            }

            const double &fitness = ga::fitness::ObjFunction::evaluate(m_organisms[i].getPerformance());

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
        for (size_t k = m_matingPoolSize; k < m_popSize; k++)
        {
            const size_t &id_1 = k % m_matingPoolSize;
            const size_t &id_2 = (k + 1) % m_matingPoolSize;

            m_organisms[k].setGenome(ga::operators::crossover::uniform(m_organisms[id_1].getGenome(), m_organisms[id_2].getGenome()));
        }
    }

    void Population::_mutation()
    {
        ga::operators::mutation::MUTATION_PROBABILITY = config::ConfigHandler<config::GA>::getGAConfig().operators.mutation_probability;

        for (size_t k = m_matingPoolSize; k < m_popSize; k++)
        {
            m_organisms[k].setGenome(ga::operators::mutation::bit_flip(m_organisms[k].getGenome()));
        }
    }

} // namespace ga