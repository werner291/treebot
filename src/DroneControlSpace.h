//
// Created by werner on 08-05-21.
//

#ifndef TREEBOT_DRONECONTROLSPACE_H
#define TREEBOT_DRONECONTROLSPACE_H

#include <Eigen/Geometry>
#include <ompl/control/SpaceInformation.h>
#include "state_spaces.h"

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
void propagateStateFromDroneControl(const ompl::base::State *from, const ompl::control::Control *control, double, ompl::base::State *to);

#endif //TREEBOT_DRONECONTROLSPACE_H
