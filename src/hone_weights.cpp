#include "genetic_algorithm/population.h"
#include "utils/config_handler.hpp"

int main(int argc, char **argv)
{
    DEBUG_LOG("Binary built in debug mode. If not intended, abort.");

    srand(time(0));

    const auto gaConfig = config::ConfigHandler<config::GA>::getGAConfig();

    const size_t &popSize = gaConfig.general.population_size;
    const size_t &matingPoolSize = gaConfig.general.mating_pool_size;
    const size_t &numberOfGenerations = gaConfig.general.generations;

    std::unique_ptr<ga::Population> newPopulation(new ga::Population(popSize, matingPoolSize));

    newPopulation->randDistInit();

    for (size_t gen = 1; gen < numberOfGenerations + 1; gen++)
    {
        CONSOLE_LOG(" -- Generation: " << gen << "\n");

        // All magic happens here !!
        newPopulation->mainLoop();

        CONSOLE_LOG("Best fitness: " << newPopulation->getBestFitness() << "\n\n");

        if (gaConfig.general.interactive_decision_tree)
            newPopulation->runIDT();

        newPopulation->refresh(gen);
    }

    CONSOLE_LOG("\033[1;33m COMPLETE \033[0m\n\n");

    CONSOLE_LOG("Optimum weights found : \n"
                << newPopulation->getBestWeights() << std::endl);
}