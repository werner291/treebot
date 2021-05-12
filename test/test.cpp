
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/control/SpaceInformation.h>
#include "state_spaces.h"
#include "DroneControlSpace.h"

#include "doctest.h"

static const double EPSILON = 0.000001;

TEST_CASE("Sampler test") {

    using namespace ompl::base;

    RealVectorBounds bounds(3);
    bounds.setHigh(10.0);
    bounds.setLow(-10.0);

    auto space(std::make_shared<PositionAndHeadingSpace>(bounds));

    ScopedState<PositionAndHeadingSpace> state(space);

    ScopedState<PositionAndHeadingSpace> state2(space);

    ScopedState<PositionAndHeadingSpace> state3(space);

    auto sampler(space->allocDefaultStateSampler());

    for (int i = 0; i < 1000; i++) {
        sampler->sampleUniform(state.get());
        CHECK(state.satisfiesBounds());

        sampler->sampleUniformNear(state2.get(), state.get(), 0.5);
        CHECK(state.satisfiesBounds());
        CHECK(state.distance(state2) <= 0.5);
    }

    ompl::RNG rng;

    for (int i = 0; i < 1000; i++) {
        sampler->sampleUniform(state.get());
        sampler->sampleUniform(state2.get());

        double t = rng.uniformReal(0.0, 1.0);

        space->interpolate(state.get(), state2.get(), t, state3.get());

        CHECK(abs(state.distance(state3) - t * state.distance(state2)) < EPSILON);
    }
}

TEST_CASE("Rotation-Heading conversion") {

    ompl::RNG rng;

    for (int i = 0; i < 1000; i++) {

        double heading = rng.uniformReal(-M_PI, M_PI);

        Eigen::Quaterniond quat(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));

        CHECK(abs(quaternionToHeading(quat) - heading) < EPSILON);

    }

}

TEST_CASE("Directed sampling test") {

    ompl::RNG rng;

    using namespace ompl::base;
    using namespace ompl::control;

    RealVectorBounds bounds(3);
    bounds.setHigh(10.0);
    bounds.setLow(-10.0);

    auto space = std::make_shared<PositionAndHeadingSpace>(bounds);
    auto controlspace = std::make_shared<DroneControlSpace>(space);

    ScopedState<PositionAndHeadingSpace> current_state(space);
    current_state.random();

    ScopedState<PositionAndHeadingSpace> destination(space);
    destination.random();

    destination->z = current_state->z;

    auto si(std::make_shared<ompl::control::SpaceInformation>(space, controlspace));
    si->setStatePropagator(std::make_shared<DronePropagator>(si.get()));

    si->setStateValidityChecker([](auto state){return true;});

    si->setDirectedControlSamplerAllocator([](const ompl::control::SpaceInformation * si) {
        return std::make_shared<DroneDirectedControlSampler>(si);
    });

    si->setup();

    auto dcs = si->allocDirectedControlSampler();

    auto deleter=[&](Control* ptr){controlspace->freeControl(ptr);};

    auto control = std::unique_ptr<Control, decltype(deleter)>(controlspace->allocControl(), deleter);

    while (current_state.distance(destination) > 0.01) {
        ScopedState<PositionAndHeadingSpace> sample(destination);

        dcs->sampleTo(control.get(), current_state.get(), sample.get());

        if (current_state.distance(destination) <= sample.distance(destination)) {
            FAIL_CHECK("Remaining distance not going down.");
            break;
        }

        space->copyState(current_state.get(), sample.get());
    }
}