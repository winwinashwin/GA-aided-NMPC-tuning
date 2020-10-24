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
    ObjFunction::ObjFunction() : m_Weights{{0.2, 0.2, 0.2, 0.2, 0.2}}
    {
    }

    double ObjFunction::evaluateImpl(model::Performance performance)
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

        for (size_t i = 0; i < iterations; i++)
        {
            ITAE_cte += (i + 1) * abs(performance.cteData[i]);
            ITAE_etheta += (i + 1) * abs(performance.ethetaData[i]);
            ITAE_vel += (i + 1) * abs(performance.velErrData[i]);

            IAE_trans += abs(performance.translationalEL[i]);
            IAE_rot += abs(performance.rotationalEL[i]);
        }

        const double metricSum = m_Weights[0] * ITAE_cte +
                                 m_Weights[1] * ITAE_etheta +
                                 m_Weights[2] * ITAE_vel +
                                 m_Weights[3] * IAE_trans +
                                 m_Weights[4] * IAE_rot;

        // We are trying to minimise the weighted average, hence goes in denominator
        double weightSum = 0;
        weightSum = std::accumulate(m_Weights.begin(), m_Weights.end(), weightSum);

        fitness += 10000 * weightSum / metricSum;

        if (fitness > 1000)
        {
            DEBUG_LOG("Abnormal fitness value");
        }

        return fitness;
    }
} // namespace ga::fitness