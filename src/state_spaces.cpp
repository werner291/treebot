//
// Created by werner on 01-05-21.
//

#include <algorithm>
#include <ompl/base/spaces/SE3StateSpace.h>
#include "state_spaces.h"

using namespace ompl::base;

double clampd(double x, double lo, double hi) {
    if (x < lo) {
        return lo;
    } else if (x > hi) {
        return hi;
    } else {
        return x;
    }
}

double wrapAngle(double x) {

    return x - std::floor( (x+M_PI)/(2.0*M_PI) ) * 2.0 * M_PI;
}


void PositionAndHeadingSpace::StateSampler::sampleUniform(ompl::base::State *state) {
    auto st = state->as<PositionAndHeadingSpace::StateType>();
    auto space = dynamic_cast<const PositionAndHeadingSpace*>(space_);

    st->x = rng_.uniformReal(space->position_bounds_.low[0], space->position_bounds_.high[0]);
    st->y = rng_.uniformReal(space->position_bounds_.low[1], space->position_bounds_.high[1]);
    st->z = rng_.uniformReal(space->position_bounds_.low[2], space->position_bounds_.high[2]);
    st->heading = rng_.uniformReal(-M_PI, M_PI);
}

void PositionAndHeadingSpace::StateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near,
                                                              double distance) {

    auto st = state->as<PositionAndHeadingSpace::StateType>();
    auto nr = near->as<PositionAndHeadingSpace::StateType>();
    auto space = dynamic_cast<const PositionAndHeadingSpace*>(space_);

    st->x = rng_.uniformReal(std::max(space->position_bounds_.low[0], nr->x - distance), std::min(space->position_bounds_.high[0], nr->x + distance));
    st->y = rng_.uniformReal(std::max(space->position_bounds_.low[1], nr->y - distance), std::min(space->position_bounds_.high[1], nr->y + distance));
    st->z = rng_.uniformReal(std::max(space->position_bounds_.low[2], nr->z - distance), std::min(space->position_bounds_.high[2], nr->z + distance));

    st->heading = wrapAngle(nr->heading + rng_.uniformReal(-distance, distance));
}

void PositionAndHeadingSpace::StateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean,
                                                           double stdDev) {
    auto st = state->as<PositionAndHeadingSpace::StateType>();
    auto mn = mean->as<PositionAndHeadingSpace::StateType>();

    st->x = rng_.gaussian(mn->x, stdDev);
    st->y = rng_.gaussian(mn->y, stdDev);
    st->z = rng_.gaussian(mn->z, stdDev);
    st->heading = rng_.gaussian(mn->heading, stdDev);

    space_->enforceBounds(st);
}

bool PositionAndHeadingSpace::satisfiesBounds(const ompl::base::State *state) const {
    auto xyzh = state->as<StateType>();
    return xyzh->x >= position_bounds_.low[0] && xyzh->x <= position_bounds_.high[0] &&
           xyzh->y >= position_bounds_.low[1] && xyzh->y <= position_bounds_.high[1] &&
           xyzh->z >= position_bounds_.low[2] && xyzh->z <= position_bounds_.high[2] &&
           xyzh->heading >= -M_PI && xyzh->heading <= M_PI;
}

void PositionAndHeadingSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const {
    auto sxyzh = source->as<StateType>();
    auto dxyzh = destination->as<StateType>();

    dxyzh->x = sxyzh->x;
    dxyzh->y = sxyzh->y;
    dxyzh->z = sxyzh->z;
    dxyzh->heading = sxyzh->heading;
}

double PositionAndHeadingSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const {
    auto s1 = state1->as<StateType>();
    auto s2 = state2->as<StateType>();
    double linear = (s1->linear() - s2->linear()).norm();
    double angular = abs(wrapAngle(s1->heading) - wrapAngle(s2->heading));
    if (angular > M_PI) {
        angular = 2.0 * M_PI - angular;
    }
    return linear + angular;
}

bool PositionAndHeadingSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const {
    auto s1 = state1->as<StateType>();
    auto s2 = state2->as<StateType>();

    return s1->x == s2->x && s1->y == s2->y && s1->z == s2->z && s1->heading == s2->heading;
}

void PositionAndHeadingSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                                          ompl::base::State *state) const {
    auto s1 = from->as<StateType>();
    auto s2 = to->as<StateType>();
    auto result = state->as<StateType>();

    result->x = s1->x * (1.0 - t) + s2-> x * t;
    result->y = s1->y * (1.0 - t) + s2-> y * t;
    result->z = s1->z * (1.0 - t) + s2-> z * t;

    double delta = s2->heading - s1->heading;

    if (delta > M_PI) {
        delta = 2.0 * M_PI - delta;
    } else if (delta < -M_PI) {
        delta = delta + 2.0 * M_PI;
    }

    result->heading = s1->heading + delta * t;

}

double *PositionAndHeadingSpace::getValueAddressAtIndex(ompl::base::State *state, unsigned int index) const {
    auto s1 = state->as<StateType>();

    switch (index) {
        case 0:
            return &s1->x;
        case 1:
            return &s1->y;
        case 2:
            return &s1->z;
        case 3:
            return &s1->heading;
        default:
            return nullptr;
    }
}

void PositionAndHeadingSpace::enforceBounds(ompl::base::State *state) const {
    auto xyzh = state->as<StateType>();

    xyzh->x = clampd(xyzh->x, position_bounds_.low[0], position_bounds_.high[0]);
    xyzh->y = clampd(xyzh->y, position_bounds_.low[1], position_bounds_.high[1]);
    xyzh->z = clampd(xyzh->z, position_bounds_.low[2], position_bounds_.high[2]);

    xyzh->heading = wrapAngle(xyzh->heading);
}

class LinearPartProjection : public ProjectionEvaluator
{
public:

    explicit LinearPartProjection(StateSpace* space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension(void) const override
    {
        return 3;
    }

    void defaultCellSizes(void) override
    {
        cellSizes_.resize(3);
        cellSizes_[0] = 0.2;
        cellSizes_[0] = 0.2;
        cellSizes_[0] = 0.2;
    }

    void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        auto st = state->as<PositionAndHeadingSpace::StateType>();
        projection(0) = st->x;
        projection(1) = st->y;
        projection(2) = st->z;
    }
};

void PositionAndHeadingSpace::registerProjections() {

    auto proj = std::make_shared<LinearPartProjection>(this);

    registerDefaultProjection(proj);

}
