//
// Created by werner on 07-05-21.
//

#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>
#include "FourierSeriesPlanner.h"
#include <random>

const unsigned int MAX_HARMONICS = 10;
const unsigned int SEGMENTS = 100;

using namespace ompl::base;
using namespace ompl::geometric;

ompl::base::PlannerStatus FourierSeriesPlanner::solve(const ompl::base::PlannerTerminationCondition &ptc) {

    State* start = pdef_->getStartState(0);

    Goal *goal = pdef_->getGoal().get();

    auto *goal_s = dynamic_cast<GoalSampleableRegion *>(goal);

    std::random_device rd;

    std::mt19937 e2(rd());

    std::uniform_real_distribution<> dist(-1, 1);

    while (!ptc) {

        ScopedState<StateSpace> goal_state(si_);

        double amplitudes[MAX_HARMONICS];

        for (double & amplitude : amplitudes) {
            amplitude = dist(e2);
        }

        auto path(std::make_shared<PathGeometric>(si_));

        bool isValid = true;

        for (int i = 0; i <= SEGMENTS && isValid; i++) {
            double t = (double) i / (double) SEGMENTS;

            State* state = si_->allocState();

            si_->getStateSpace()->interpolate(start, goal_state.get(), t, state);

            path->append(state);

            if (i >= 1) {
                isValid &= si_->checkMotion(path->getState(i-1), path->getState(i));
            }
        }

        if (isValid) {

            pdef_->addSolutionPath(
                    path
            );

        }

    }

    return ompl::base::PlannerStatus();
}

