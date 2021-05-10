//
// Created by werner on 01-05-21.
//

#ifndef TREEBOT_STATE_SPACES_H
#define TREEBOT_STATE_SPACES_H

static const double DISTANCE_ANGULAR_WEIGHT = M_1_PI;

#include <algorithm>
#include <ompl/base/StateSpace.h>
#include "conversions.h"
#include "look_forward.h"

class PositionAndHeadingSpace : public ompl::base::StateSpace {

    class StateSampler : public ompl::base::StateSampler
    {

    public:
        void sampleUniform(ompl::base::State *state) override;

        void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

        void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;

        explicit StateSampler(const PositionAndHeadingSpace *space) : ompl::base::StateSampler(space) {}
    };

    ompl::base::RealVectorBounds position_bounds_;
public:
    explicit PositionAndHeadingSpace(const ompl::base::RealVectorBounds &positionBounds) : position_bounds_(positionBounds) {
        assert(position_bounds_.high.size() == 3 && position_bounds_.low.size() == 3);
    }

    class StateType : public ompl::base::State
    {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double heading = 0.0;

        double getX() const {
            return x;
        }

        void setX(double x) {
            StateType::x = x;
        }

        double getY() const {
            return y;
        }

        void setY(double y) {
            StateType::y = y;
        }

        double getZ() const {
            return z;
        }

        void setZ(double z) {
            StateType::z = z;
        }

        double getHeading() const {
            return heading;
        }

        void setHeading(double heading) {
            StateType::heading = heading;
        }

        Eigen::Vector3d linear() const {
            return Eigen::Vector3d(x,y,z);
        }

        Eigen::Quaterniond rotation() const {
            return Eigen::Quaterniond(Eigen::AngleAxisd(heading, Eigen::Vector3d(0,0,1)));
        }

        void setXYZH(double x, double y, double z, double heading) {
            setX(x);
            setY(y);
            setZ(z);
            setHeading(heading);
        }

    };

public:
    unsigned int getDimension() const override {
        return 4;
    }

    double getMaximumExtent() const override {
        return (Eigen::Vector3d(position_bounds_.high.data()) - Eigen::Vector3d(position_bounds_.low.data())).norm() + M_PI;
    }

    double getMeasure() const override {
        return position_bounds_.getVolume() * M_PI;
    }

    void enforceBounds(ompl::base::State *state) const override;

    bool satisfiesBounds(const ompl::base::State *state) const override;

    void copyState(ompl::base::State *destination, const ompl::base::State *source) const override;

    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

    bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override;

    void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                     ompl::base::State *state) const override;

    ompl::base::StateSamplerPtr allocDefaultStateSampler() const override {
        return std::make_shared<StateSampler>(this);
    }

    ompl::base::State *allocState() const override {
        return new StateType();
    }

    void freeState(ompl::base::State *state) const override {
        delete state->as<StateType>();
    }

    double *getValueAddressAtIndex(ompl::base::State *state, unsigned int index) const override;

    void registerProjections() override;

    void printState(const ompl::base::State *state, std::ostream &out) const override
    {
        auto st = state->as<StateType>();
        out << "State instance [" << st->x << ", " <<  st->y << ", " << st->z << ", h:" << st->heading <<']' << std::endl;
    }
};


#endif //TREEBOT_STATE_SPACES_H
