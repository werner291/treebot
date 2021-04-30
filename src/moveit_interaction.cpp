//
// Created by werner on 30-04-21.
//

#include "conversions.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/PointStamped.h>

std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> initPlanningSceneMonitor(std::shared_ptr<tf2_ros::Buffer> tfBuf) {
    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tfBuf);
    psm->providePlanningSceneService();
    psm->startStateMonitor();
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    tfBuf->lookupTransform("base_link", "map", ros::Time(0.0), ros::Duration(5.0));
    psm->waitForCurrentRobotState(ros::Time::now(), 5.0);

    return psm;
}

void displayMultiDoFTrajectory(std::unique_ptr<moveit_visual_tools::MoveItVisualTools> &visual_tools,
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