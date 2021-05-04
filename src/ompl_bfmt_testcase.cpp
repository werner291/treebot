#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <algorithm>
#include <Eigen/Geometry>

namespace ob = ompl::base;
namespace og = ompl::geometric;


class LookForwardValidator : public ompl::base::MotionValidator {

public:
    explicit LookForwardValidator(const ompl::base::SpaceInformationPtr &si) : MotionValidator(si) {}

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override {

        auto ss1 = s1->as<ob::SE3StateSpace::StateType>();
        auto ss2 = s2->as<ob::SE3StateSpace::StateType>();

        Eigen::Vector3d forward_local(0.0, 1.0, 0.0);

        Eigen::Vector3d delta_linear(ss2->getX() - ss1->getX(), ss2->getY() - ss1->getY(), ss2->getZ() - ss1->getZ());
        double distance = delta_linear.norm();

        if (distance > std::numeric_limits<double>::epsilon()) {

            auto fwd_1 = Eigen::Quaterniond(ss1->rotation().w, ss1->rotation().x, ss1->rotation().y, ss1->rotation().z) * forward_local;
            auto fwd_2 = Eigen::Quaterniond(ss2->rotation().w, ss2->rotation().x, ss2->rotation().y, ss2->rotation().z) * forward_local;

            return (fwd_1.dot(delta_linear) / distance > 0.5) && (fwd_2.dot(delta_linear) / distance > 0.5);
        } else {
            return true;
        }
    };

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override {
        abort(); // Not implemented.
    }
};

int main(int argc, char **argv) {

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10.0);
    bounds.setHigh(10.0);
    auto space = std::make_shared<ob::SE3StateSpace>();
    space->setBounds(bounds);

    auto si(std::make_shared<ob::SpaceInformation>(space));

    auto mv(std::make_shared<LookForwardValidator>(si));
    si->setMotionValidator(std::static_pointer_cast<ob::MotionValidator>(mv));

    si->setStateValidityChecker([](const ob::State *st) {
        return true;
    });
    si->setup();

    ompl::base::ScopedState<ob::SE3StateSpace> start(space);
    start->setXYZ(0.0, -5.0, 0.0);
    start->rotation().setAxisAngle(0.0,0.0,1.0,0.0);

    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal->setXYZ(0.0, 5.0, 0.0);
    goal->rotation().setAxisAngle(0.0,0.0,1.0,M_PI);

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal, 0.5);

    auto planner(std::make_shared<og::BFMT>(si));
    planner->setProblemDefinition(pdef);

    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

    ompl::base::PlannerData pd(si);
    planner->og::BFMT::getPlannerData(pd);

    if (solved) {
        // print the path to screen
        const ob::PathPtr path = pdef->getSolutionPath();

        std::cout << "Found solution:" << std::endl;
        path->print(std::cout);

        auto states = path->as<ompl::geometric::PathGeometric>()->getStates();

        for (int i = 0; i < states.size()-1; i++) {
            printf("Motion valid? %s reverse: %s\n", mv->checkMotion(states[i],states[i+1]) ? "Yes" : "No", mv->checkMotion(states[i+1],states[i]) ? "Yes" : "No");
        }
    }

    return 0;
}