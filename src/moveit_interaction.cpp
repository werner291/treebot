//
// Created by werner on 30-04-21.
//

#include "DroneControlSpace.h"
#include "moveit_interaction.h"
#include "conversions.h"
#include "look_forward.h"
#include "state_spaces.h"
#include <algorithm>
#include <geometry_msgs/PointStamped.h>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <queue>
#include <tf2_ros/transform_listener.h>

std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>
initPlanningSceneMonitor(const std::shared_ptr<tf2_ros::Buffer> &tfBuf,
                         double first_message_delay = 1.0) {
    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            "robot_description", tfBuf);
    psm->providePlanningSceneService();
    psm->startStateMonitor();
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startPublishingPlanningScene(
            planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    tfBuf->lookupTransform("base_link", "map", ros::Time(0.0),
                           ros::Duration(5.0));
    psm->waitForCurrentRobotState(ros::Time::now(), 5.0);

    // Put in a delay to make sure we've got an octomap and TF info
    ros::Duration(first_message_delay).sleep();

    return psm;
}

void displayMultiDoFTrajectory(
        std::unique_ptr<moveit_visual_tools::MoveItVisualTools> &visual_tools,
        moveit_msgs::RobotTrajectory &robotTrajectory,
        const moveit::core::RobotState &current_state) {
    moveit_msgs::DisplayTrajectory dt;
    dt.trajectory.push_back(robotTrajectory);

    moveit_msgs::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(current_state, robot_state_msg);

    moveit_msgs::DisplayTrajectory display_trajectory_msg;
    display_trajectory_msg.model_id = current_state.getRobotModel()->getName();
    display_trajectory_msg.trajectory.resize(1);
    display_trajectory_msg.trajectory[0] = robotTrajectory;
    display_trajectory_msg.trajectory_start = robot_state_msg;

    visual_tools->publishTrajectoryPath(display_trajectory_msg);
}

ompl::base::ScopedState<PositionAndHeadingSpace>
moveItStateToPositionAndHeading(
        const std::shared_ptr<PositionAndHeadingSpace> &space,
        moveit::core::RobotState &current_state) {
    ompl::base::ScopedState<PositionAndHeadingSpace> start(space);

    double *floating_joint_positions = current_state.getVariablePositions();
    start->as<PositionAndHeadingSpace::StateType>()->x =
            floating_joint_positions[0];
    start->as<PositionAndHeadingSpace::StateType>()->y =
            floating_joint_positions[1];
    start->as<PositionAndHeadingSpace::StateType>()->z =
            floating_joint_positions[2];

    // Note: Eigen's quaternions are [w,x,y,z], but the floating joint has
    // [x,y,z,w]
    Eigen::Quaterniond rot(
            floating_joint_positions[6], floating_joint_positions[3],
            floating_joint_positions[4], floating_joint_positions[5]);

    Eigen::Vector3d facing = rot * Eigen::Vector3d::UnitY();

#pragma clang diagnostic push
#pragma ide diagnostic ignored                                                 \
    "ArgumentSelectionDefects" // It seems to be running fine.
    start->as<PositionAndHeadingSpace::StateType>()->heading =
            -atan2(facing.x(), facing.y());
#pragma clang diagnostic pop
    return start;
}

void visualizePlannerStates(
        std::unique_ptr<moveit_visual_tools::MoveItVisualTools> &visual_tools,
        ompl::base::PlannerData &pd) {

    for (int i = 0; i < pd.numVertices(); ++i) {

        auto v = pd.getVertex(i);
        auto st = v.getState()->as<PositionAndHeadingSpace::StateType>();

        geometry_msgs::Pose pose;
        pose.position.x = st->x;
        pose.position.y = st->y;
        pose.position.z = st->z;

        auto rot = Eigen::Quaterniond(Eigen::AngleAxisd(
                st->heading +
                M_PI / 2.0 /* Arrow points down X-axis, rotate to compensate*/,
                Eigen::Vector3d(0, 0, 1)));
        pose.orientation.x = rot.x();
        pose.orientation.y = rot.y();
        pose.orientation.z = rot.z();
        pose.orientation.w = rot.w();

        visual_tools->publishArrow(pose, rviz_visual_tools::GREEN,
                                   rviz_visual_tools::LARGE);
    }

    visual_tools->trigger();
}

MoveitStateChecker::MoveitStateChecker(
        const ompl::base::SpaceInformationPtr &si,
        const planning_scene::PlanningScenePtr &ps,
        const moveit::core::RobotState &templateState)
        : StateValidityChecker(si), ps_(ps), template_state_(templateState) {}

bool MoveitStateChecker::isValid(const ompl::base::State *st) const {
    auto st1 = st->as<PositionAndHeadingSpace::StateType>();

    // Since the floor in CopelliaSim isn't infinite,
    // I feel this is appropriate or the planner might try to pass underneath it.
    if (st1->getZ() < 0.0) {
        return false;
    }

    moveit::core::RobotState rs(template_state_);

    Eigen::Quaterniond rot(
            Eigen::AngleAxisd(st1->getHeading(), Eigen::Vector3d(0, 0, 1)));

    double positions[] = {st1->getX(), st1->getY(), st1->getZ(), rot.x(),
                          rot.y(), rot.z(), rot.w()};

    rs.setJointPositions("world_joint", positions);

    rs.update();

    return ps_->isStateValid(rs);
}


bool isTrajectoryStillValid(const std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> &psm,
                            const std::vector<ompl::base::State *> &states,
                            const std::shared_ptr<ompl::base::SpaceInformation> &si) {
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

        valid &= segmentValid;
    }
    return valid;
}
