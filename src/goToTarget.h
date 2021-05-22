//
// Created by werner on 17-05-21.
//

#ifndef TREEBOT_GOTOTARGET_H
#define TREEBOT_GOTOTARGET_H

#include <mutex>
#include "DroneControlSpace.h"
#include "state_spaces.h"
#include "moveit_interaction.h"
#include "conversions.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <memory>
#include <queue>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include "goToTarget.h"

void goToTarget(std::mutex &targets_mutex, const std::vector<Eigen::Vector3d> &latest_targets,
                const std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> &psm,
                std::_MakeUniq<moveit_visual_tools::MoveItVisualTools>::__single_object &visual_tools,
                std::shared_ptr<ompl::control::SpaceInformation> &si,
                std::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager> &tem, bool taskCompleted);
#endif //TREEBOT_GOTOTARGET_H
