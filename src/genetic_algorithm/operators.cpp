#include "genetic_algorithm/operators.h"

namespace ga::operators::mutation
{
    ga::core::Genome bitFlip(const ga::core::Genome &genome, double mutationProbability)
    {
        ga::core::Genome newGenome = genome;

        for (auto &chrom : newGenome.chromosomes)
            for (size_t i = 0; i < chrom.genes.size(); i++)
                if ((rand() / double(RAND_MAX)) < mutationProbability)
                    chrom.genes[i] = !chrom.genes[i];

        return newGenome;
    }
} // namespace ga::operators::mutation

namespace ga::operators::crossover
{
    ga::core::Genome uniform(const ga::core::Genome &parent_1, const ga::core::Genome &parent_2, double bias)
    {
        // Copy gene of parent
        ga::core::Genome offspring_1 = parent_1;
        // Genome offspring_2 = parent_2;

        const size_t chrom_size = parent_1.chromosomes.size();
        const size_t n_genes = parent_1.chromosomes[0].genes.size();

        for (size_t i = 0; i < chrom_size; i++)
        {
            for (size_t j = 0; j < n_genes; j++)
            {
                if ((rand() / double(RAND_MAX)) < bias)
                {
                    offspring_1.chromosomes[i].genes[j] = parent_1.chromosomes[i].genes[j];
                    // offspring_2.chromosomes[i].unit[j] = parent_2.chromosomes[i].unit[j];
                }
                else
                {
                    offspring_1.chromosomes[i].genes[j] = parent_2.chromosomes[i].genes[j];
                    // offspring_2.chromosomes[i].unit[j] = parent_1.chromosomes[i].unit[j];
                }
            }
        }
        return offspring_1;
    }

} // namespace ga::operators::crossover