#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const std::string PLANNING_GROUP = "whole_body";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "treebot_controller", 0);
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    move_group.setPlannerId("RRTConnectkConfigDefault");

    move_group.setWorkspace(-100.0, -100.0, -100.0, 100.0, 100.0, 100.0);

    move_group.setStartStateToCurrentState();

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("bubblebot/base_link");

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    auto joints = move_group.getJoints();

    for (auto & joint : joints) {
        ROS_INFO_NAMED("tutorial", "Start: [%s]", joint.c_str());
    }


    move_group.setPlannerId("RRTstarkConfigDefault");

//    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//    planning_scene_interface.addCollisionObjects()

    std::vector<double> values;
    values.push_back(-2.0);
    values.push_back(2.0);
    values.push_back(0.5);
    values.push_back(0.0);
    values.push_back(0.0);
    values.push_back(0.0);
    values.push_back(1.0);
    move_group.setJointValueTarget("world_joint", values);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line %s", success ? "SUCCESS" : "FAILED");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
    visual_tools.trigger();

    move_group.move();

    ros::waitForShutdown();


    return 0;
}