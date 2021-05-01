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
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/Constraint.h>
#include <algorithm>

#include "look_forward.h"
#include "conversions.h"
#include "moveit_interaction.h"
#include "state_spaces.h"

static const std::string PLANNING_GROUP = "whole_body";

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace rvt = rviz_visual_tools;


//
//class DroneConstraint : public ob::Constraint {
//public:
//    DroneConstraint() : Constraint(7,1) {}
//
//    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override {
//
//        Eigen::Quaterniond orientation(x[3],x[4],x[5],x[6]);
//
//        Eigen::Vector3d up(0,0,1.0);
//
//        out[0] = (orientation * up).z() - 1.0;
//
//    }
//};


int main(int argc, char** argv) {
    ros::init(argc, argv, "treebot_controller", 0);
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto tfBuf = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0), true);
    auto tfList = std::make_shared<tf2_ros::TransformListener>(*tfBuf);
    ros::Duration(1.0).sleep();

    auto psm = initPlanningSceneMonitor(tfBuf);

    auto visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("map", "markers_viz", psm);
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();

    moveit_msgs::MotionPlanResponse response;
    moveit_msgs::RobotTrajectory robotTrajectory;

    {
        planning_scene_monitor::LockedPlanningSceneRW ps(psm);

//        auto constraint = std::make_shared<DroneConstraint>();

        ob::RealVectorBounds bounds(3);
        for (int d = 0; d < 3; ++d) {
            bounds.setLow(d, d == 2 ? 0.0 : -5.0);
            bounds.setHigh(d, 5.0);
        }
        auto space = std::make_shared<PositionAndHeadingSpace>(bounds);

        auto current_state = ps->getCurrentState();

        ompl::base::ScopedState<PositionAndHeadingSpace> start(space);

        double *floating_joint_positions = current_state.getVariablePositions();
        start->as<PositionAndHeadingSpace::StateType>()->x = floating_joint_positions[0];
        start->as<PositionAndHeadingSpace::StateType>()->y = floating_joint_positions[1];
        start->as<PositionAndHeadingSpace::StateType>()->z = floating_joint_positions[2];

        start->as<PositionAndHeadingSpace::StateType>()->heading =
                Eigen::Quaterniond(&floating_joint_positions[3]).angularDistance(Eigen::Quaterniond::Identity());

        ob::ScopedState<PositionAndHeadingSpace> goal(space);
        goal->as<PositionAndHeadingSpace::StateType>()->x = 0.0;
        goal->as<PositionAndHeadingSpace::StateType>()->y = 0.0;
        goal->as<PositionAndHeadingSpace::StateType>()->z = 0.2;
        goal->as<PositionAndHeadingSpace::StateType>()->heading = 0.0;

        auto si(std::make_shared<ob::SpaceInformation>(space));

//        space->setSpaceInformation(si.get());

//        auto dmv = std::make_shared<ob::DiscreteMotionValidator>(si);
//        auto mv(std::make_shared<LookForwardValidator>(si, dmv));
//        si->setMotionValidator(std::static_pointer_cast<ob::MotionValidator>(mv));

        si->setStateValidityChecker([&ps, current_state](const ob::State *st) {

            auto st1 = st->as<PositionAndHeadingSpace::StateType>();

            // Since the floor in CopelliaSim isn't infinite,
            // I feel this is appropriate or the planner might try to pass underneath it.
            if (st1->getZ() < 0.0) {
                return false;
            }

            moveit::core::RobotState rs(current_state);

            Eigen::Quaterniond rot(Eigen::AngleAxisd(st1->getHeading(), Eigen::Vector3d(0,0,1)));

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

            return ps->isStateValid(rs);
        });
        si->setup();

        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal, 0.5);

//        auto planner(std::make_shared<og::BITstar>(si));
        auto planner(std::make_shared<og::FMT>(si));
        planner->setProblemDefinition(pdef);

        std::cout << "Starting planner..." << std::endl;
        ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

        if (solved) {
            // print the path to screen
            const ob::PathPtr path = pdef->getSolutionPath();

            std::cout << "Found solution:" << std::endl;
            path->print(std::cout);

            robotTrajectory = trajectoryToMoveit(path);
        } else {
            ROS_ERROR("OMPL planner error: %s", solved.asString().c_str());
            return 1;
        }

        displayMultiDoFTrajectory(visual_tools, robotTrajectory, current_state);

        visual_tools->trigger();
    }

    auto tem = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(psm->getRobotModel(),
                                                                                          psm->getStateMonitor(), true);
    tem->push(robotTrajectory, "drone_controller");

    tem->executeAndWait();

    ros::waitForShutdown();

    return 0;
}