//
// Created by werner on 08-05-21.
//

#ifndef TREEBOT_DRONECONTROLSPACE_H
#define TREEBOT_DRONECONTROLSPACE_H

#include <Eigen/Geometry>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/DirectedControlSampler.h>
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

class DroneDirectedControlSampler : public ompl::control::DirectedControlSampler {
public:
    explicit DroneDirectedControlSampler(const ompl::control::SpaceInformation *space) : DirectedControlSampler(space) {}

    unsigned int sampleTo(ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest) override;

    unsigned int
    sampleTo(ompl::control::Control *control, const ompl::control::Control *previous, const ompl::base::State *source,
             ompl::base::State *dest) override;
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

    void printControl(const ompl::control::Control *control, std::ostream &out = std::cout) const override {
        auto st = control->as<DroneControl>();
        out << "Control instance [" << st->x << ", " <<  st->y << ", " << st->z << ", h:" << st->rotational <<']' << std::endl;
    }


};

/**
 * Implementation of a state propagator for DroneControl and PositionAndHeadingSpace
 */
class DronePropagator : public ompl::control::StatePropagator {
public:
    DronePropagator(ompl::control::SpaceInformation *si) : StatePropagator(si) {}

    void propagate(const ompl::base::State *from, const ompl::control::Control *control, double duration,
                   ompl::base::State *to) const override;

};
#endif //TREEBOT_DRONECONTROLSPACE_H
