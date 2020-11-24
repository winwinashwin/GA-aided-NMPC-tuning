#ifndef GA_CORE_H_
#define GA_CORE_H_

#include "pcheaders.h"
#include "mpc_lib/mpc.h"

/**
 * Core components of the genetic algorithm. Includes Genome, chromosomes etc
 */
namespace ga::core
{
    struct Chromosome
    {
        // Constructor
        Chromosome(double lb, double ub);

        /**
         * Encode weight into the chromosome
         * 
         * @param weight: Weight to be encoded
         */
        void encodeWeight(double weight);

        /**
         * Decode weight from chromosome
         * 
         * @return Decoded weight
         */
        double decodeWeight() const;

        /// Maximum length of bitset for each weight
        ///
        /// Increase this value for higher precision, but in cost of computation
        static const size_t __MAX_LEN = 20;
        // Lower bound for the weight
        double LB;
        /// Upperbound for the weight
        double UB;
        /// Bitset for stroring the binary data
        std::bitset<__MAX_LEN> genes;
    };

    struct Genome
    {
        /// Constructor
        Genome();

        /// Prettify output on console
        operator std::string() const;

        /**
         * Encode weights into the genome
         * 
         * @param weights: weights to be encoded
         */
        void encode(const mpc::Params::Weights &weights);

        /**
         * Decode genome to get weights
         * 
         * @return Decoded weights
         */
        mpc::Params::Weights decode() const;

        /**
         * Add a chromosome to genome
         * 
         * @param lb: Lowerbound for the weight to be encoded
         * @param ub: Upperbound for the weight to be encoded
         */
        void addChoromosome(double lb, double ub);

        /// Chromosomes in the Genome
        std::vector<Chromosome> chromosomes;
    };
} // namespace ga::core
#endif
