//
// Created by werner on 08-05-21.
//

#ifndef TREEBOT_DRONECONTROLSPACE_H
#define TREEBOT_DRONECONTROLSPACE_H

#include <Eigen/Geometry>
#include <ompl/control/SpaceInformation.h>
#include "state_spaces.h"

const double LINEAR_PART_TANGENT = 0.5;

class DroneControl : public ompl::control::Control {
public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rotational = 0.0;
};

/**
 * Sampler that produces Control settings that ensure that the robot translates only in directions covered by the sensor.
 */
class DroneControlSampler : public ompl::control::ControlSampler {
public:
    explicit DroneControlSampler(const ompl::control::ControlSpace *space) : ControlSampler(space) {}

    void sample(ompl::control::Control *control) override;
};

/**
 * Control space that corresponds to the DroneControl Control implementation.
 */
class DroneControlSpace : public ompl::control::ControlSpace {

public:
    explicit DroneControlSpace(const ompl::base::StateSpacePtr &stateSpace) : ControlSpace(stateSpace) {}

    unsigned int getDimension() const override;

    ompl::control::Control *allocControl() const override;

    void freeControl(ompl::control::Control *control) const override;

    void copyControl(ompl::control::Control *destination, const ompl::control::Control *source) const override;

    bool equalControls(const ompl::control::Control *control1, const ompl::control::Control *control2) const override;

    void nullControl(ompl::control::Control *control) const override;

    ompl::control::ControlSamplerPtr allocDefaultControlSampler() const override;


};

/**
 * Implementation of a state propagator for DroneControl and PositionAndHeadingSpace
 */
class DronePropagator : public ompl::control::StatePropagator {
public:
    DronePropagator(ompl::control::SpaceInformation *si) : StatePropagator(si) {}

    void propagate(const ompl::base::State *from, const ompl::control::Control *control, double duration,
                   ompl::base::State *to) const override;

//    virtual bool steer(const ompl::base::State * from, const ompl::base::State * to, ompl::control::Control * result,
//                       double & duration) const
//    {
//        auto frm = dynamic_cast<const PositionAndHeadingSpace::StateType*>(from);
//        auto dst = dynamic_cast<const PositionAndHeadingSpace::StateType*>(to);
//
//        auto delta_local = frm->rotation().inverse() * (dst->linear() - frm->linear());
//        auto linear_distance = delta_local
//
//
//        auto delta_local_unit = delta_local.normalized();
//
//        if (abs(delta_local.x()) < 0.3 )
//
//
//        auto ctrl = dynamic_cast<DroneControl*>(result);
//
//        return false;
//    }
//
//    /** \brief Return true if the steer() function has been implemented */
//    virtual bool canSteer() const
//    {
//        return true;
//    }

};
#endif //TREEBOT_DRONECONTROLSPACE_H
