//
// Created by werner on 08-05-21.
//

#include <memory>
#include <queue>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include "DroneControlSpace.h"



void DroneControlSampler::sample(ompl::control::Control *control) {
    auto c = dynamic_cast<DroneControl *>(control);



    Eigen::Vector3d linear_part(rng_.uniformReal(-LINEAR_PART_TANGENT, LINEAR_PART_TANGENT), 1.0,
                                rng_.uniformReal(-LINEAR_PART_TANGENT, LINEAR_PART_TANGENT));
    linear_part.normalize();
    linear_part *= rng_.uniformReal(0.0, 1.0);

    c->x = linear_part.x();
    c->y = linear_part.y();
    c->z = linear_part.z();

    c->rotational = rng_.uniformReal(-1.0, 1.0);
}

void DroneControlSpace::copyControl(ompl::control::Control *destination, const ompl::control::Control *source) const {
    auto d = dynamic_cast<DroneControl *>(destination);
    auto s = dynamic_cast<const DroneControl *>(source);

    d->x = s->x;
    d->y = s->y;
    d->z = s->z;
    d->rotational = s->rotational;
}

bool
DroneControlSpace::equalControls(const ompl::control::Control *control1, const ompl::control::Control *control2) const {
    auto c1 = dynamic_cast<const DroneControl *>(control1);
    auto c2 = dynamic_cast<const DroneControl *>(control2);

    return c1->x == c2->x && c1->y == c2->y && c1->z == c2->z && c1->rotational == c2->rotational;
}

void DroneControlSpace::nullControl(ompl::control::Control *control) const {
    auto c = dynamic_cast<DroneControl *>(control);
    c->x = 0.0;
    c->y = 0.0;
    c->z = 0.0;
    c->rotational = 0.0;
}

ompl::control::ControlSamplerPtr DroneControlSpace::allocDefaultControlSampler() const {
    return std::make_shared<DroneControlSampler>(this);
}

ompl::control::Control *DroneControlSpace::allocControl() const {
    return static_cast<ompl::control::Control *>(new DroneControl());
}

void DroneControlSpace::freeControl(ompl::control::Control *control) const {
    delete dynamic_cast<DroneControl *>(control);
}

unsigned int DroneControlSpace::getDimension() const {
    return 4;
}

void DronePropagator::propagate(const ompl::base::State *from, const ompl::control::Control *control, double duration,
                                ompl::base::State *to) const {
    auto frm = dynamic_cast<const PositionAndHeadingSpace::StateType*>(from);
    auto result = dynamic_cast<PositionAndHeadingSpace::StateType*>(to);
    auto ctrl = dynamic_cast<const DroneControl*>(control);

    auto rot = frm->rotation();
    auto linear_part = rot * Eigen::Vector3d(ctrl->x, ctrl->y, ctrl->z);

    result->x = frm->x + linear_part.x() * duration;
    result->y = frm->y + linear_part.y() * duration;
    result->z = frm->z + linear_part.z() * duration;
    result->heading = frm->heading + ctrl->rotational * duration;
}
