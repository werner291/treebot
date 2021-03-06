//
// Created by werner on 30-04-21.
//

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
#include "look_forward.h"

#include "state_spaces.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool LookForwardValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {

    auto ss1 = s1->as<PositionAndHeadingSpace::StateType>();
    auto ss2 = s2->as<PositionAndHeadingSpace::StateType>();

    Eigen::Vector3d forward_local(0.0, 1.0, 0.0);

    Eigen::Vector3d delta_linear(ss2->getX() - ss1->getX(), ss2->getY() - ss1->getY(), ss2->getZ() - ss1->getZ());
    double distance = delta_linear.norm();

    if (distance > std::numeric_limits<double>::epsilon()) {
        auto fwd_1 = ss1->rotation() * forward_local;
        //auto fwd_2 = ss2->rotation() * forward_local;
        return super_->checkMotion(s1, s2) &&
                (fwd_1.dot(delta_linear) / distance > 0.8);// && (fwd_2.dot(delta_linear) / distance > 0.5);
    } else {
        return super_->checkMotion(s1, s2);
    }
}

bool LookForwardValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                       std::pair<ompl::base::State *, double> &lastValid) const {
    auto ss1 = s1->as<PositionAndHeadingSpace::StateType>();
    auto ss2 = s2->as<PositionAndHeadingSpace::StateType>();

    Eigen::Vector3d forward_local(0.0, 1.0, 0.0);

    Eigen::Vector3d delta_linear(ss2->getX() - ss1->getX(), ss2->getY() - ss1->getY(), ss2->getZ() - ss1->getZ());
    double distance = delta_linear.norm();

    if (distance > std::numeric_limits<double>::epsilon()) {
        auto fwd_1 = ss1->rotation() * forward_local;
        //auto fwd_2 = ss2->rotation() * forward_local;
        return super_->checkMotion(s1, s2, lastValid) &&
               (fwd_1.dot(delta_linear) / distance > 0.8);// && (fwd_2.dot(delta_linear) / distance > 0.5);
    } else {
        return super_->checkMotion(s1, s2, lastValid);
    }
}