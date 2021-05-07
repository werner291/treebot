#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/plan_execution/plan_execution.h>
#include <tf2_ros/transform_listener.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <algorithm>

#include <queue>
#include <memory>
#include <utility>

#include "look_forward.h"
#include "conversions.h"
#include "moveit_interaction.h"
#include "state_spaces.h"

static const std::string PLANNING_GROUP = "whole_body";

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace rvt = rviz_visual_tools;

class MoveitStateChecker : public ob::StateValidityChecker {

    const planning_scene::PlanningScenePtr ps_;
    const moveit::core::RobotState &template_state_;
public:
    MoveitStateChecker(const ompl::base::SpaceInformationPtr &si,
                       const planning_scene::PlanningScenePtr &ps,
                       const moveit::core::RobotState &templateState) : StateValidityChecker(si), ps_(ps),
                                                                        template_state_(templateState) {}

public:
    bool isValid(const ompl::base::State *st) const override {
        auto st1 = st->as<PositionAndHeadingSpace::StateType>();

        // Since the floor in CopelliaSim isn't infinite,
        // I feel this is appropriate or the planner might try to pass underneath it.
        if (st1->getZ() < 0.0) {
            return false;
        }

        moveit::core::RobotState rs(template_state_);

        Eigen::Quaterniond rot(Eigen::AngleAxisd(st1->getHeading(), Eigen::Vector3d(0, 0, 1)));

        double positions[] = {
                st1->getX(),
                st1->getY(),
                st1->getZ(),
                rot.x(),
                rot.y(),
                rot.z(),
                rot.w()
        };

        rs.setJointPositions("world_joint", positions);

        rs.update();

        return ps_->isStateValid(rs);
    }

};

planning_scene::PlanningScenePtr
snapshotPlanningScene(const std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> &psm);

bool isTrajectoryStillValid(const std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> &psm,
                            const std::vector<ob::State *> &states,
                            std::shared_ptr<ob::SpaceInformation> &si) {
    planning_scene::PlanningScenePtr ps;
    {
        // Keep in a block to drop the lock.
        ps = planning_scene_monitor::LockedPlanningSceneRO(psm)->diff();
    }
    moveit::core::RobotState state = ps->getCurrentState();
    si->setStateValidityChecker(std::make_shared<MoveitStateChecker>(si, ps, state));
    si->setup();
    bool valid= true;
    for (int i = 0; i < states.size() - 1; i++) {
        bool segmentValid = si->checkMotion(states[i], states[i + 1]);

//        ROS_INFO("Segment valid: %s", segmentValid ? "yes" : "no");

        valid &= segmentValid;
    }
    return valid;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "treebot_controller", 0);
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto tfBuf = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0), true);
    auto tfList = std::make_shared<tf2_ros::TransformListener>(*tfBuf);
    auto psm = initPlanningSceneMonitor(tfBuf, 1.0);

    // Make sure to put this after initializing the TF buffer and planning scene monitor
    // to allow it to receive the first TF messages and octomap.

    auto visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("map", "markers_viz", psm);
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();

    moveit_msgs::MotionPlanResponse response;


    ob::RealVectorBounds bounds(3);
    for (int d = 0; d < 3; ++d) {
        bounds.setLow(d, d == 2 ? 0.0 : -5.0);
        bounds.setHigh(d, 5.0);
    }
    auto space = std::make_shared<PositionAndHeadingSpace>(bounds);

    std::shared_ptr<ob::SpaceInformation> si(std::make_shared<ob::SpaceInformation>(space));

    std::shared_ptr<ob::DiscreteMotionValidator> dmv = std::make_shared<ob::DiscreteMotionValidator>(si);

    std::shared_ptr<LookForwardValidator> mv = std::make_shared<LookForwardValidator>(si, dmv);

    si->setMotionValidator(std::static_pointer_cast<ob::MotionValidator>(mv));

    auto tem = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(psm->getRobotModel(),
                                                                                          psm->getStateMonitor(), true);
    tem->setAllowedStartTolerance(0.1);

    bool taskCompleted = false;

    do {

        //        auto planner(std::make_shared<og::BITstar>(si));
//    auto planner(std::make_shared<og::RRTConnect>(si));
//    auto planner(std::make_shared<og::FMT>(si));
//    planner->setNumSamples(10000);
//    planner->setHeuristics(true);
    auto planner(std::make_shared<og::InformedRRTstar>(si));
//        auto planner(std::make_shared<og::KPIECE1>(si));

        planning_scene::PlanningScenePtr ps = snapshotPlanningScene(psm);

        moveit::core::RobotState current_state = ps->getCurrentState();

        si->setStateValidityChecker(std::make_shared<MoveitStateChecker>(si, ps, current_state));
        si->setup();

        ompl::base::ScopedState<PositionAndHeadingSpace> start = moveItStateToPositionAndHeading(space, current_state);

        start.print();

        ob::ScopedState<PositionAndHeadingSpace> goal(space);
        goal->setXYZH(0.0, 0.0, 0.2, 0.0);

        std::shared_ptr<ob::ProblemDefinition> pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal, 0.01);

        auto opt(std::make_shared<ob::PathLengthOptimizationObjective>(si));

        pdef->setOptimizationObjective(opt);

        planner->clear();
        planner->setProblemDefinition(pdef);


        ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);

//        ob::PlannerData pd(si);
//        planner->og::KPIECE1::getPlannerData(pd);
//        visualizePlannerStates(visual_tools, pd);
//
//        visual_tools->trigger();

        if (solved == ob::PlannerStatus::EXACT_SOLUTION) {

            // print the path to screen
            const ob::PathPtr path = pdef->getSolutionPath();

            auto states = path->as<ompl::geometric::PathGeometric>()->getStates();

            {
                moveit::core::RobotState state = ps->getCurrentState();
                bool valid= true;
                for (int i = 0; i < states.size() - 1; i++) {
                    bool segmentValid = si->checkMotion(states[i], states[i + 1]);

                    ROS_INFO("Segment valid: %s", segmentValid ? "yes" : "no");

                    valid &= segmentValid;
                }
            }

            path->print(std::cout);

            moveit_msgs::RobotTrajectory robotTrajectory = trajectoryToMoveit(path);

            displayMultiDoFTrajectory(visual_tools, robotTrajectory, current_state);

            tem->push(robotTrajectory);

            bool done = false;
            tem->execute([&done, &taskCompleted](auto status) {
                ROS_INFO("Execution finished.");

                if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED) {
                    taskCompleted = true;
                }

                done = true;
            });

            ros::Rate rate(5.0);

            while (!done) {

                rate.sleep();

                bool valid = isTrajectoryStillValid(psm, states, si);

                if (!valid) {
                    ROS_WARN("Trajectory invalidated!");
                    tem->stopExecution();
                } else {
                    ROS_INFO("Trajectory valid!");
                }
            }

        } else {
            ROS_ERROR("OMPL planner error: %s. Retrying.", solved.asString().c_str());
        }

        // Wait a bit to let the robot settle.
        ros::Duration(2.0).sleep();

    } while (!taskCompleted);

    ROS_INFO("Task completed, waiting for shutdown signal.");

    ros::waitForShutdown();

    return 0;
}

planning_scene::PlanningScenePtr
snapshotPlanningScene(const std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> &psm) {
    planning_scene::PlanningScenePtr ps;
    {
        // Keep in a block to drop the lock.
        ps = planning_scene_monitor::LockedPlanningSceneRO(psm)->diff();
    }
    return ps;
}

