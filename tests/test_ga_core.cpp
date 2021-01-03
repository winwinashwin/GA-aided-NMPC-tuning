#include "genetic_algorithm/core.h"
#include "mpc_lib/mpc.h"

#include <gtest/gtest.h>

static ::testing::AssertionResult IsBetweenInclusive(double val, double a, double b)
{
    if ((val >= a) && (val <= b))
        return ::testing::AssertionSuccess();
    else
        return ::testing::AssertionFailure()
               << val << " is outside the range " << a << " to " << b;
}

TEST(GaCoreTestSuite, testGenome)
{
    ga::core::Genome genome;

    genome.addChoromosome(0.1, 100.0);
    genome.addChoromosome(0.1, 100.0);
    genome.addChoromosome(0.1, 100.0);
    genome.addChoromosome(0.01, 100.0);
    genome.addChoromosome(0.01, 100.0);
    genome.addChoromosome(0.01, 100.0);
    genome.addChoromosome(0.01, 100.0);

    mpc::Params::Weights w = {10.0, 0.1, 43.2, 12.634, 52.009, 100.0, 99.99};

    genome.encode(w);

    auto wDecoded = genome.decode();

    EXPECT_TRUE(IsBetweenInclusive(wDecoded.vel, 9.99, 10.01));
    EXPECT_TRUE(IsBetweenInclusive(wDecoded.cte, 0.099, 0.101));
    EXPECT_TRUE(IsBetweenInclusive(wDecoded.etheta, 43.19, 43.21));
    EXPECT_TRUE(IsBetweenInclusive(wDecoded.omega, 12.633, 12.635));
    EXPECT_TRUE(IsBetweenInclusive(wDecoded.acc, 52.008, 52.01));
    EXPECT_TRUE(IsBetweenInclusive(wDecoded.omega_d, 99.999, 100.001));
    EXPECT_TRUE(IsBetweenInclusive(wDecoded.acc_d, 99.989, 99.991));
}