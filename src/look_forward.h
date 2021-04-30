//
// Created by werner on 30-04-21.
//

#ifndef TREEBOT_LOOK_FORWARD_H
#define TREEBOT_LOOK_FORWARD_H

#include <memory>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/DiscreteMotionValidator.h>

class LookForwardValidator : public ompl::base::MotionValidator {

public:
    LookForwardValidator(const ompl::base::SpaceInformationPtr &si,
                         std::shared_ptr<ompl::base::DiscreteMotionValidator> super) : MotionValidator(si), super_(std::move(super)) {}

private:
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override;

    std::shared_ptr<ompl::base::DiscreteMotionValidator> super_;


};

#endif //TREEBOT_LOOK_FORWARD_H
