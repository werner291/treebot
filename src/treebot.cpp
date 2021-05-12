#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <algorithm>

#include <queue>
#include <memory>
#include <utility>

#include "look_forward.h"
#include "conversions.h"
#include "moveit_interaction.h"
#include "state_spaces.h"
#include "DroneControlSpace.h"

static const std::string PLANNING_GROUP = "whole_body";

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

namespace rvt = rviz_visual_tools;

planning_scene::PlanningScenePtr
snapshotPlanningScene(const std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> &psm);

std::shared_ptr<oc::SpaceInformation> initSpaceInformation();

int main(int argc, char **argv) {
    ros::init(argc, argv, "treebot_controller", 0);
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto tfBuf = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0), true);
    auto tfList = std::make_shared<tf2_ros::TransformListener>(*tfBuf);
    auto psm = initPlanningSceneMonitor(tfBuf, 1.0);

    auto visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("map", "markers_viz", psm);

    auto si = initSpaceInformation();

    si->setDirectedControlSamplerAllocator([](const oc::SpaceInformation * si) {
        return std::make_shared<oc::SimpleDirectedControlSampler>(si, 10);
//        return std::make_shared<DroneDirectedControlSampler>(si);
    });

    auto tem = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(psm->getRobotModel(),
                                                                                          psm->getStateMonitor(), true);
    tem->setAllowedStartTolerance(0.5);

    bool taskCompleted = false;

    do {

        visual_tools->deleteAllMarkers();
        visual_tools->trigger();

//        auto planner(std::make_shared<oc::SST>(si));
        auto planner(std::make_shared<oc::PDST>(si));


        planning_scene::PlanningScenePtr ps = snapshotPlanningScene(psm);

        moveit::core::RobotState current_state = ps->getCurrentState();

        si->setStateValidityChecker(std::make_shared<MoveitStateChecker>(si, ps, current_state));
        si->setup();

        auto space = std::dynamic_pointer_cast<PositionAndHeadingSpace>(si->getStateSpace());

        auto start = moveItStateToPositionAndHeading(space, current_state);

        ob::ScopedState<PositionAndHeadingSpace> goal(space);
        goal->setXYZH(0.0, 0.0, 0.4, 0.0);

        std::shared_ptr<ob::ProblemDefinition> pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal, 0.01);

        auto opt(std::make_shared<ob::PathLengthOptimizationObjective>(si));

        pdef->setOptimizationObjective(opt);

        planner->setProblemDefinition(pdef);

        auto planner_start_time = ros::Time::now();

//        ob::PlannerStatus solved = planner->solve(ob::PlannerTerminationCondition([&pdef, planner_start_time]() {
//            printf("Difference: %f", pdef->getSolutionDifference());
//            return pdef->getSolutionDifference() < 0.5 && (ros::Time::now() - planner_start_time) > ros::Duration(20.0);
//        }));
        ob::PlannerStatus solved = planner->ob::Planner::solve(20.0);
//
//        ob::PlannerData pd(si);
//        planner->oc::PDST::getPlannerData(pd);
//        visualizePlannerStates(visual_tools, pd);
//        visual_tools->trigger();

        ROS_INFO("Planner finished with status: %s", solved.asString().c_str());

        if (solved == ob::PlannerStatus::EXACT_SOLUTION || solved == ob::PlannerStatus::APPROXIMATE_SOLUTION) {

            // print the path to screen
            const ob::PathPtr path = pdef->getSolutionPath();

            auto states = path->as<ompl::geometric::PathGeometric>()->getStates();

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
                }
            }

        } else {
            ROS_ERROR("OMPL planner error: %s. Retrying.", solved.asString().c_str());
        }

        // Wait a bit to let the robot settle.
        ros::Duration(2.0).sleep();

    } while (!taskCompleted);

    ROS_INFO("\\033[32m Task completed, waiting for shutdown signal.");

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

std::shared_ptr<oc::SpaceInformation> initSpaceInformation() {
    ob::RealVectorBounds bounds(3);
    for (int d = 0; d < 3; ++d) {
        bounds.setLow(d, d == 2 ? 0.0 : -5.0);
        bounds.setHigh(d, 5.0);
    }
    auto space = std::make_shared<PositionAndHeadingSpace>(bounds);
    auto controlspace = std::make_shared<DroneControlSpace>(space);
    auto si(std::make_shared<oc::SpaceInformation>(space, controlspace));
    si->setStatePropagator(std::make_shared<DronePropagator>(si.get()));
    return si;
}

