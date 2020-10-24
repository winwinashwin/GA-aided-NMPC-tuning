#include "genetic_algorithm/fitness.h"

/**
 * Normalize data
 */
static void normalize(std::vector<double> &array)
{
    const auto [min, max] = std::minmax_element(begin(array), end(array));

    for (size_t i = 0; i < array.size(); i++)
    {
        array[i] = (array[i] - *min) / (*max - *min);
    }
}

namespace ga::fitness
{
    double evaluate(model::Performance performance)
    {
        double fitness = 0.0;
        const size_t &iterations = performance.cteData.size();

        normalize(performance.velErrData);
        normalize(performance.ethetaData);
        normalize(performance.cteData);

        normalize(performance.translationalEL);
        normalize(performance.rotationalEL);

        // We take Integral Time Absolute Error(ITAE) for the error values and 
        // Integral Absolute Error(IAE) for the energy losses(EL)
        double ITAE_cte = 0.0, ITAE_etheta = 0.0, ITAE_vel = 0.0;
        double IAE_trans = 0.0, IAE_rot = 0.0;

        const double w1 = 4.0;
        const double w2 = 3.0;
        const double w3 = 2.0;
        const double w4 = 2.0;
        const double w5 = 2.0;

        for (size_t i = 0; i < iterations; i++)
        {
            ITAE_cte += (i + 1) * abs(performance.cteData[i]);
            ITAE_etheta += (i + 1) * abs(performance.ethetaData[i]);
            ITAE_vel += (i + 1) * abs(performance.velErrData[i]);

            IAE_trans += abs(performance.translationalEL[i]);
            IAE_rot += abs(performance.rotationalEL[i]);
        }

        const double &weightedAvg = (w1 * ITAE_cte + w2 * ITAE_etheta + w3 * ITAE_vel + w4 * IAE_trans + w5 * IAE_rot) / (w1 + w2 + w3 + w4 + w5);

        // We are trying to minimise the weighted average, hence goes in denominator
        fitness += 10000 / weightedAvg;

        if (fitness > 1000)
        {
            DEBUG_LOG("Abnormal fitness value");
        }
        return fitness;
    }
} // namespace ga::fitness