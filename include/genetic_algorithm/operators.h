#ifndef GA_OPERATORS_H_
#define GA_OPERATORS_H_

#include "pcheaders.h"

#include "genetic_algorithm/core.h"

/**
 * Mutation operators
 */
namespace ga::operators::mutation
{
    static double MUTATION_PROBABILITY = 0.03;

    /**
     * Select one or more random bits and flip them. Used in binary encoded GAs.
     * 
     * @param genome: Genome to mutate
     * 
     * @return New mutated genome
     */
    ga::core::Genome bit_flip(const ga::core::Genome &genome);

} // namespace ga::operators::mutation

/**
 * Crossover operators
 */
namespace ga::operators::crossover
{
    /**
     * Do a uniform crossover between two parents
     * 
     * In a uniform crossover, we don’t divide the chromosome into segments, rather we treat each gene separately. 
     * In this, we essentially flip a coin for each chromosome to decide whether or not it’ll be included in the off-spring. 
     * We can also bias the coin to one parent, to have more genetic material in the child from that parent.
     * 
     * @param parent1: First parent involved in crossover
     * @param parent2: Second parent involved in crossover
     * @param bias: b/w 0 and 1, indicates how biased the crossover is towards the first parent.
     * 
     * @return New crossed Genome
     */
    ga::core::Genome uniform(const ga::core::Genome &parent1, const ga::core::Genome &parent2, double bias = 0.5);
} // namespace ga::operators::crossover

#endif
