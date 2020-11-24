#include "genetic_algorithm/core.h"

namespace ga::core
{
    Chromosome::Chromosome(double lb, double ub) : LB(lb),
                                                                 UB(ub)
    {
    }

    /**
     * The mapping from a binary string <b0, b1, b2 ... b(n-1)> into a 
     * real number x from the range [lb, ub] is straightforward and is
     * completed in two steps
     * 
     * (i) Convert binary string <b0, b1, b2 .. b(n-1)> from base 2
     * to base 10. Let this number be x'.
     * 
     * (ii) Corresponding real number x is given by:
     * 
     *      x = lb + x' * ((ub - lb) / (2^(n) - 1))
     */
    void Chromosome::encodeWeight(double weight)
    {
        // Scale the bitset representation such that value is within
        // the lb and ub of the chromosome.
        const double factor = (UB - LB) / (pow(2, __MAX_LEN) - 1);
        genes = std::bitset<__MAX_LEN>(static_cast<int>((weight - LB) / factor));
    }

    double Chromosome::decodeWeight() const
    {
        // Scale back the representation such that value is within
        // the lb and ub of the chromosome.
        const auto value = genes.to_ulong();
        double factor = (UB - LB) / (pow(2, __MAX_LEN) - 1);

        return (LB + static_cast<double>(value) * factor);
    }

    /************************************************************************/

    Genome::Genome()
    {
        chromosomes.reserve(7); // Reserve space for 7 chromosomes
    }

    Genome::operator std::string() const
    {
        std::string result("[ ");
        for (const auto &chrom : chromosomes)
        {
            result += chrom.genes.to_string();
            result += " ";
        }
        result += "]";

        return result;
    }

    void Genome::encode(const mpc::Params::Weights &weights)
    {
        chromosomes[0].encodeWeight(weights.vel);
        chromosomes[1].encodeWeight(weights.cte);
        chromosomes[2].encodeWeight(weights.etheta);
        chromosomes[3].encodeWeight(weights.omega);
        chromosomes[4].encodeWeight(weights.acc);
        chromosomes[5].encodeWeight(weights.omega_d);
        chromosomes[6].encodeWeight(weights.acc_d);
    }

    mpc::Params::Weights Genome::decode() const
    {
        mpc::Params::Weights weights;

        weights.vel = chromosomes[0].decodeWeight();
        weights.cte = chromosomes[1].decodeWeight();
        weights.etheta = chromosomes[2].decodeWeight();
        weights.omega = chromosomes[3].decodeWeight();
        weights.acc = chromosomes[4].decodeWeight();
        weights.omega_d = chromosomes[5].decodeWeight();
        weights.acc_d = chromosomes[6].decodeWeight();

        return weights;
    }

    void Genome::addChoromosome(double lb, double ub)
    {
        chromosomes.emplace_back(lb, ub);
    }
} // namespace ga::core