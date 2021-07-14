//
// Created by werner on 11-06-21.
//

#ifndef TREEBOT_REALTIMETRAJECTORYEXECUTOR_H
#define TREEBOT_REALTIMETRAJECTORYEXECUTOR_H

#include <thread>
#include <mutex>
#include "moveit_interaction.h"
#include "conversions.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <memory>
#include <queue>
#include <boost/graph/directed_graph.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

struct TrajectoryPoint {
    moveit::core::RobotState state;
    ros::Time at_time;
};

class RealtimeTrajectoryExecutor {
    tf2_ros::Buffer& tf;
    std::vector<TrajectoryPoint> trajectory;
    ros::NodeHandle& nh;

    std::thread background_thread;

    std::mutex mutex;

public:

    RealtimeTrajectoryExecutor(ros::NodeHandle &nh, tf2_ros::Buffer &tf);

    void start();

    void run();

    void updateTrajectory(const std::vector<TrajectoryPoint> new_trajectory);

};

#endif //TREEBOT_REALTIMETRAJECTORYEXECUTOR_H
