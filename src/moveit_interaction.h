//
// Created by werner on 30-04-21.
//

#ifndef TREEBOT_MOVEIT_INTERACTION_H
#define TREEBOT_MOVEIT_INTERACTION_H

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/buffer.h>

std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> initPlanningSceneMonitor(std::shared_ptr<tf2_ros::Buffer> tfBuf);

void displayMultiDoFTrajectory(std::unique_ptr<moveit_visual_tools::MoveItVisualTools> &visual_tools,
                               moveit_msgs::RobotTrajectory &robotTrajectory,
                               const moveit::core::RobotState &current_state);

#endif //TREEBOT_MOVEIT_INTERACTION_H
