#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/kinematic_constraints/utils.h>
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

//    robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
//    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    planning_scene_monitor::LockedPlanningSceneRO ps(psm);

    std::vector<double> values;

    auto robot_model = ps->getRobotModel();
    auto jmg = ps->getRobotModel()->getJointModelGroup("whole_body");

    moveit::core::RobotState goal_state(robot_model);
    std::vector<double> joint_values = { -2.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0 };
    goal_state.setJointGroupPositions(jmg, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);

    planning_interface::MotionPlanRequest req;
    req.group_name = PLANNING_GROUP;
    req.goal_constraints.push_back(joint_goal);

    req.start_state.multi_dof_joint_state.joint_names.push_back("world_joint");
    geometry_msgs::Transform xfm;
    xfm.translation.x = 0.0;
    xfm.translation.y = 0.0;
    xfm.translation.z = 0.0;
    xfm.rotation.x = 0.0;
    xfm.rotation.y = 0.0;
    xfm.rotation.z = 0.0;
    xfm.rotation.w = 1.0;
    req.start_state.multi_dof_joint_state.transforms.push_back(xfm);

    req.workspace_parameters.min_corner.x = -100.0;
    req.workspace_parameters.min_corner.y = 0.0;
    req.workspace_parameters.min_corner.z = -100.0;

    req.workspace_parameters.max_corner.x = 100.0;
    req.workspace_parameters.max_corner.y = 100.0;
    req.workspace_parameters.max_corner.z = 100.0;

    req.allowed_planning_time = 1.0;

    planning_interface::MotionPlanResponse res;

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, nh.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
            ss << cls << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                             << "Available plugins: " << ss.str());
    }

    planning_interface::PlanningContextPtr context =
            planner_instance->getPlanningContext(ps, req, res.error_code_);

    context->solve(res);

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    auto tem = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(psm->getRobotModel(), psm->getStateMonitor(), true);

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    tem->pushAndExecute(response.trajectory, "drone_controller");

    tem->waitForExecution();

//    collision_detection::CollisionRequest collision_request;
//    collision_detection::CollisionResult collision_result;
//    planning_scene.checkSelfCollision(collision_request, collision_result);
//    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
//
//    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
//    kinematic_state->setToDefaultValues();
//    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("whole_body");
//
//    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
//
//    std::vector<double> joint_values;
//    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
//    for (std::size_t i = 0; i < joint_names.size(); ++i)
//    {
//        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//    }


//
//    kinematic_state->setToRandomPositions(joint_model_group);
//    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("base_link");
//
//    /* Print end-effector pose. Remember that this is in the model frame */
//    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
//    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

//
//    // The :planning_interface:`MoveGroupInterface` class can be easily
//    // setup using just the name of the planning group you would like to control and plan for.
//    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//
//    move_group.setPlannerId("RRTConnectkConfigDefault");
//
//    move_group.setWorkspace(-100.0, -100.0, -100.0, 100.0, 100.0, 100.0);
//
//    move_group.setStartStateToCurrentState();
//
//    namespace rvt = rviz_visual_tools;
//    moveit_visual_tools::MoveItVisualTools visual_tools("bubblebot/base_link");
//
//    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//    text_pose.translation().z() = 1.0;
//    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
//    visual_tools.trigger();
//
//    auto joints = move_group.getJoints();
//
//    for (auto & joint : joints) {
//        ROS_INFO_NAMED("tutorial", "Start: [%s]", joint.c_str());
//    }
//
//
//    move_group.setPlannerId("RRTstarkConfigDefault");
//
////    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
////    planning_scene_interface.addCollisionObjects()
//
//    std::vector<double> values;
//    values.push_back(-2.0);
//    values.push_back(2.0);
//    values.push_back(0.5);
//    values.push_back(0.0);
//    values.push_back(0.0);
//    values.push_back(0.0);
//    values.push_back(1.0);
//    move_group.setJointValueTarget("world_joint", values);
//
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line %s", success ? "SUCCESS" : "FAILED");
//    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//    visual_tools.publishTrajectoryLine(my_plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
//    visual_tools.trigger();
//
//    move_group.move();
//
//    ros::waitForShutdown();


ROS_INFO("Terminate");
    return 0;
}