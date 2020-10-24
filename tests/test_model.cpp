#include "pcheaders.h"
#include "model/differential_drive.h"

#include <gtest/gtest.h>

TEST(ModelTestSuite, testModel)
{
    model::State state = {-8.0, 1.5, -0.6, 0.0, 0.0, 0.0};

    ASSERT_DOUBLE_EQ(state.x, -8.0);
    ASSERT_DOUBLE_EQ(state.y, 1.5);
    ASSERT_DOUBLE_EQ(state.theta, -0.6);
    ASSERT_DOUBLE_EQ(state.linVel, 0.0);
    ASSERT_DOUBLE_EQ(state.angVel, 0.0);
    ASSERT_DOUBLE_EQ(state.throttle, 0.0);

    model::DifferentialDrive dDriveModel;
    dDriveModel.setSampleTime(0.1);

    dDriveModel.setInitState(state);

    dDriveModel.step(1.0, 2.5);

    model::State firstState = dDriveModel.getState();

    ASSERT_DOUBLE_EQ(firstState.x, -8.0);
    ASSERT_DOUBLE_EQ(firstState.y, 1.5);
    ASSERT_DOUBLE_EQ(firstState.theta, -0.35);
    ASSERT_DOUBLE_EQ(firstState.linVel, 1.0);
    ASSERT_DOUBLE_EQ(firstState.angVel, 2.5);
    ASSERT_DOUBLE_EQ(firstState.throttle, 10.0);

    dDriveModel.reset();

    model::State secondState = dDriveModel.getState();

    ASSERT_DOUBLE_EQ(secondState.x, -8.0);
    ASSERT_DOUBLE_EQ(secondState.y, 1.5);
    ASSERT_DOUBLE_EQ(secondState.theta, -0.6);
    ASSERT_DOUBLE_EQ(secondState.linVel, 0.0);
    ASSERT_DOUBLE_EQ(secondState.angVel, 0.0);
    ASSERT_DOUBLE_EQ(secondState.throttle, 0.0);
}