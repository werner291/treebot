//
// Created by werner on 30-04-21.
//

#include "look_forward.h"
#include <utility>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include "conversions.h"


moveit_msgs::RobotTrajectory trajectoryToMoveit(const ompl::base::PathPtr &path) {
    moveit_msgs::RobotTrajectory rtraj;
    rtraj.multi_dof_joint_trajectory.joint_names.push_back("world_joint");
    for (auto & st : path->as<ompl::geometric::PathGeometric>()->getStates()) {

        trajectory_msgs::MultiDOFJointTrajectoryPoint current_point = stateToTrajectoryPoint(st);

        if (rtraj.multi_dof_joint_trajectory.points.empty()) {
            current_point.time_from_start = ros::Duration(0.0);
        } else {
            auto last_point = rtraj.multi_dof_joint_trajectory.points.back();

            auto p1 = current_point.transforms[0].translation;
            auto p2 = last_point.transforms[0].translation;

            double length = (Eigen::Vector3d(p1.x, p1.y, p1.z) - Eigen::Vector3d(p2.x, p2.y, p2.z)).norm();
            double speed = 0.1;

            current_point.time_from_start = last_point.time_from_start + ros::Duration(length / speed);
        }

        rtraj.multi_dof_joint_trajectory.points.push_back(current_point);
    }
    return rtraj;
}

trajectory_msgs::MultiDOFJointTrajectoryPoint stateToTrajectoryPoint(ompl::base::State *st) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint mdjtp;
    geometry_msgs::Transform tf;

    auto st1 = st->as<PositionAndHeadingSpace::StateType>();

    tf.translation.x = st1->getX();
    tf.translation.y = st1->getY();
    tf.translation.z = st1->getZ();

    auto rot = st1->rotation();

    tf.rotation.x = rot.x();
    tf.rotation.y = rot.y();
    tf.rotation.z = rot.z();
    tf.rotation.w = rot.w();

    mdjtp.transforms.push_back(tf);

    return mdjtp;
}