#ifndef GA_FITNESS_H_
#define GA_FITNESS_H_

#include "pcheaders.h"

#include "model/differential_drive.h"

/**
 * Fitness function
 */
namespace ga::fitness
{
    /**
     * Evaluate the fitness of an individual
     * 
     * @param performance: Performance/response of the individual in the MPC control loop
     * 
     * @return Fitness value
     */
    double evaluate(model::Performance performance);
} // namespace ga::fitness
#endif
