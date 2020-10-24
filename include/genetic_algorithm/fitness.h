#ifndef GA_FITNESS_H_
#define GA_FITNESS_H_

#include "pcheaders.h"

#include "model/differential_drive.h"

/**
 * Fitness function
 */
namespace ga::fitness
{
    class ObjFunction
    {
    public:
        ObjFunction(const ObjFunction &) = delete;
        /**
         * Evaluate the fitness of an individual
         * 
         * @param performance: Performance/response of the individual in the MPC control loop
         * 
         * @return Fitness value
         */
        static double evaluate(model::Performance performance)
        {
            return get().evaluateImpl(performance);
        }

    private:
        ObjFunction();

        static ObjFunction &get()
        {
            static ObjFunction s_Instance;
            return s_Instance;
        }

        double evaluateImpl(model::Performance performance);

        std::array<double, 5> m_Weights;
    };
} // namespace ga::fitness
#endif
