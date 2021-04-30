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
#include <tf2_ros/transform_listener.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include "look_forward.h"
#include "conversions.h"
#include "moveit_interaction.h"

static const std::string PLANNING_GROUP = "whole_body";

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace rvt = rviz_visual_tools;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "treebot_controller", 0);
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto tfBuf = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0), true);
    auto tfList = std::make_shared<tf2_ros::TransformListener>(*tfBuf);
    ros::Duration(1.0).sleep();

    auto psm = initPlanningSceneMonitor(tfBuf);

    auto visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("map", "markers_viz",psm);
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();

    moveit_msgs::MotionPlanResponse response;
    moveit_msgs::RobotTrajectory robotTrajectory;

    {
        planning_scene_monitor::LockedPlanningSceneRW ps(psm);

        auto current_state = ps->getCurrentState();
        auto space = std::make_shared<ob::SE3StateSpace>();

        ob::RealVectorBounds bounds(3);
        bounds.setLow(-5.0);
        bounds.setHigh(5.0);
        space->setBounds(bounds);

        auto start = floatingJointPositionsToSE3(space, current_state.getJointPositions("world_joint"));

        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(-2.0, 2.0, 0.5);
        goal->rotation().setIdentity();

        auto si(std::make_shared<ob::SpaceInformation>(space));

        auto dmv = std::make_shared<ob::DiscreteMotionValidator>(si);
        auto mv(std::make_shared<LookForwardValidator>(si, dmv));
        si->setMotionValidator(std::static_pointer_cast<ob::MotionValidator>(mv));

        si->setStateValidityChecker([&ps, current_state](const ob::State* st) {

            // Since the floor in CopelliaSim isn't infinite,
            // I feel this is appropriate or the planner might try to pass underneath it.
            if (st->as<ob::SE3StateSpace::StateType>()->getZ() < 0.0) {
                return false;
            }

            moveit::core::RobotState rs(current_state);
            setFloatingJointFromSE3(st, rs, "world_joint");
            rs.update();

            return ps->isStateValid(rs);
        });
        si->setup();

        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal, 0.5);

//        auto planner(std::make_shared<og::BITstar>(si));
        auto planner(std::make_shared<og::FMT>(si));
        planner->setProblemDefinition(pdef);

        ob::PlannerStatus solved = planner->ob::Planner::solve(60.0);

        if (solved)
        {
            // print the path to screen
            const ob::PathPtr path = pdef->getSolutionPath();

            std::cout << "Found solution:" << std::endl;
            path->print(std::cout);

            robotTrajectory = trajectoryToMoveit(path);
        } else {
            ROS_ERROR("OMPL planner error: %s", solved.asString().c_str());
        }

        displayMultiDoFTrajectory(visual_tools, robotTrajectory, current_state);

        visual_tools->trigger();
    }

    auto tem = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(psm->getRobotModel(), psm->getStateMonitor(), true);
    tem->push(robotTrajectory, "drone_controller");

    tem->executeAndWait();

    ros::waitForShutdown();

    return 0;
}

