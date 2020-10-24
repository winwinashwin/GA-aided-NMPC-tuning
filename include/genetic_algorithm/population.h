#ifndef GA_POPULATION_H_
#define GA_POPULATION_H_

#include "pcheaders.h"

#include "genetic_algorithm/organism.h"
#include "utils/progress_bar.hpp"
#include "utils/config_handler.hpp"

namespace ga
{
    /**
     * Representative of a population of orgaisnms/potential solutions
     */
    class Population
    {
    public:
        /**
         * Constructor
         * 
         * @param size: Size of the population
         * @param matingPoolSize: Size of the mating pool
         */
        Population(const size_t &size, const size_t &matingPoolSize);

        /**
         * Assign weights to all organisms in the population using uniform random distribution
         */
        void randDistInit();

        /**
         * The main loop
         * 
         * Processes such as selection, crossover and mutation happen here
         */
        void mainLoop();

        /**
         * Get best fitness of the current population
         * 
         * @return Best fitness value
         */
        double getBestFitness() const;

        /**
         * Refresh/reset the population for the next evolutionary cycle
         * 
         * @param genCount: The count of generation
         */ 
        void refresh(const size_t &genCount);

        /**
         * Get best set of weights in the population
         * 
         * @return Stringified weights, ready to be printed to the console
         */
        std::string getBestWeights() const;

        void runIDT() const;

    private:
        /**
         * Update the fitness values of the population
         */
        void _updateFitnessVals();

        /**
         * Perform selection and crossover
         */
        void _crossover();

        /**
         * Mutate the progenies
         */
        void _mutation();

        const size_t m_popSize;
        const size_t m_matingPoolSize;

        std::vector<ga::Organism> m_organisms;

        /// Progress bar for some nice console output
        ProgressBar m_pBar;
    };
} // namespace ga
#endif