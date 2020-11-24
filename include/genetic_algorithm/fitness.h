#ifndef GA_FITNESS_H_
#define GA_FITNESS_H_

#include "pcheaders.h"
#include "model/differential_drive.h"

/**
 * Fitness function
 */
namespace ga::fitness
{
    typedef std::array<double, 5> darr5_t;

    class ObjFunction
    {
    public:
        /// Delete copy constructor
        ObjFunction(const ObjFunction &) = delete;

        /**
         * Evaluate the fitness of an individual
         * 
         * @param performance: Performance/response of the individual in the MPC control loop
         * 
         * @return Fitness value
         */
        static double evaluate(const model::Performance performance)
        {
            return get()._evaluateImpl(performance);
        }

        static void interactiveDCT(const model::Performance &performance)
        {
            return get()._interactiveDctImpl(performance);
        }

    private:
        ObjFunction();

        static ObjFunction &get()
        {
            static ObjFunction s_Instance;
            return s_Instance;
        }

        double _evaluateImpl(const model::Performance &performance);

        void _interactiveDctImpl(const model::Performance &performance);

        darr5_t _getMetrics(model::Performance performance);

        darr5_t m_Weights;
        darr5_t m_prevMetrics;

        bool m_started, m_terminated;
        const double m_deltaN;
    };
} // namespace ga::fitness
#endif
