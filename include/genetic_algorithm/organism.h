#ifndef GA_ORGANISM_H_
#define GA_ORGANISM_H_

#include "pcheaders.h"
#include "model/differential_drive.h"
#include "model/base_organism.h"
#include "genetic_algorithm/core.h"
#include "utils/json_logger.hpp"

namespace ga
{
    /**
     * Representative of an organism in the Genetic Algorithm
     */
    class Organism : public model::BaseOrganism<config::GA>
    {
    public:
        /// Constructor
        Organism();

        /**
         * Get fitness of organism
         * 
         * @return Fitness
         */
        double getFitness() const;

        /**
         * Get weights represented by the organism
         * 
         * @return Representative weights
         */
        mpc::Params::Weights getWeights() const;

        /**
         * Get the genome of the organism
         * 
         * @return Genome of the organism
         */
        ga::core::Genome getGenome() const;

        /**
         * Set the fitness value of the orgaism
         * 
         * @param fitness: Fitness value
         */
        void setFitness(double fitness);

        /**
         * Assign weights to the orgaism
         * 
         * @param weights: Weights to be assigned
         */
        void setWeights(const mpc::Params::Weights &weights);

        /**
         * Set the genome
         * 
         * @param genome: Genome
         */
        void setGenome(const ga::core::Genome &genome);

        /**
         * Save the organism as the best in a population
         */
        void saveAsBest(size_t genCount);

    private:
        /// Genome of individual
        ga::core::Genome m_genome;

        /// Fitness of this individual
        double m_fitness;
    };

} // namespace ga
#endif