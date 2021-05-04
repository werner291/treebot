//
// Created by werner on 30-04-21.
//

#ifndef TREEBOT_MOVEIT_INTERACTION_H
#define TREEBOT_MOVEIT_INTERACTION_H

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/buffer.h>
#include <queue>
#include <algorithm>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <geometry_msgs/PointStamped.h>
#include "state_spaces.h"
#include "look_forward.h"
#include "conversions.h"

std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>
initPlanningSceneMonitor(const std::shared_ptr<tf2_ros::Buffer>& tfBuf, double first_message_delay);

void displayMultiDoFTrajectory(std::unique_ptr<moveit_visual_tools::MoveItVisualTools> &visual_tools,
                               moveit_msgs::RobotTrajectory &robotTrajectory,
                               const moveit::core::RobotState &current_state);


void visualizePlannerStates(std::unique_ptr<moveit_visual_tools::MoveItVisualTools> &visual_tools,
                            ompl::base::PlannerData &pd);

ompl::base::ScopedState<PositionAndHeadingSpace>
moveItStateToPositionAndHeading(const std::shared_ptr<PositionAndHeadingSpace> &space,
                                moveit::core::RobotState &current_state);

#endif //TREEBOT_MOVEIT_INTERACTION_H
