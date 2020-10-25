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
    ObjFunction::ObjFunction() : m_Weights{{0.2, 1.0, 0.4, 0.2, 0.2}},
                                 m_started(false),
                                 m_terminated(false),
                                 m_deltaN(0.05)
    {
    }

    darr5_t ObjFunction::_getMetrics(model::Performance performance)
    {
        darr5_t metrics = {0.0, 0.0, 0.0, 0.0, 0.0};

        const size_t &iterations = performance.cteData.size();

        normalize(performance.velErrData);
        normalize(performance.ethetaData);
        normalize(performance.cteData);

        normalize(performance.translationalEL);
        normalize(performance.rotationalEL);

        // We take Integral Time Absolute Error(ITAE) for the error values and
        // Integral Absolute Error(IAE) for the energy losses(EL)
        for (size_t i = 0; i < iterations; i++)
        {
            metrics[0] += (i + 1) * abs(performance.cteData[i]);
            metrics[1] += (i + 1) * abs(performance.ethetaData[i]);
            metrics[2] += (i + 1) * abs(performance.velErrData[i]);

            metrics[3] += abs(performance.translationalEL[i]);
            metrics[4] += abs(performance.rotationalEL[i]);
        }

        return metrics;
    }

    double ObjFunction::_evaluateImpl(const model::Performance &performance)
    {
        double fitness = 0.0;

        const darr5_t metrics = _getMetrics(performance);

        for (size_t i = 0; i < 5; i++)
        {
            fitness += m_Weights[i] * metrics[i];
        }

        // We are trying to minimise the weighted average, hence goes in denominator
        double weightSum = 0;
        weightSum = std::accumulate(m_Weights.begin(), m_Weights.end(), weightSum);

        fitness = 10000 * weightSum / fitness;

        if (fitness > 1000)
        {
            DEBUG_LOG("Abnormal fitness value");
        }

        return fitness;
    }

    void ObjFunction::_interactiveDctImpl(const model::Performance &performance)
    {
        if (m_terminated)
            return;

        const darr5_t currMetrics = _getMetrics(performance);

        if (!m_started)
        {
            m_started = true;
        }
        else
        {
            darr5_t improv;
            size_t n, j;

            for (size_t i = 0; i < 5; i++)
                improv[i] = 100 * (1 - currMetrics[i] / m_prevMetrics[i]);

            CONSOLE_LOG(
                "Improvements from previous generation: \n"
                << "    (j = 0) ITAE cte     : " << improv[0] << "\n"
                << "    (j = 1) ITAE etheta  : " << improv[1] << "\n"
                << "    (j = 2) ITAE vel     : " << improv[2] << "\n"
                << "    (j = 3) IAE EL trans : " << improv[3] << "\n"
                << "    (j = 4) IAE EL rot   : " << improv[4] << "\n");
            CONSOLE_LOG("Number of improvements: (0 to terminate IDT) ");

            while (true)
            {
                std::cin >> n;

                if (n < 0 && n > 5)
                {
                    CONSOLE_LOG("Input should be less than 5\n");
                }
                else
                {
                    break;
                }
            }

            if (n == 0)
                m_terminated = true;
            else
                CONSOLE_LOG("Enter j values: ");

            while (n--)
            {
                std::cin >> j;
                m_Weights[j] += m_deltaN;
            }
            CONSOLE_LOG("\n");
        }

        m_prevMetrics = currMetrics;
    }
} // namespace ga::fitness