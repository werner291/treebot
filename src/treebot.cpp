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
#include <tf2_ros/transform_listener.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

static const std::string PLANNING_GROUP = "whole_body";

namespace ob = ompl::base;
namespace og = ompl::geometric;

planning_interface::PlannerManagerPtr initPlannerManager(const ros::NodeHandle &nh, const moveit::core::RobotModelConstPtr& robot_model);
namespace rvt = rviz_visual_tools;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "treebot_controller", 0);
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();


    auto tfBuf = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0), true);
    auto tfList = std::make_shared<tf2_ros::TransformListener>(*tfBuf);
    ros::Duration(1.0).sleep();

    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tfBuf);
    psm->providePlanningSceneService();
    psm->startStateMonitor();
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    tfBuf->lookupTransform("base_link", "map", ros::Time(0.0), ros::Duration(5.0));
    psm->waitForCurrentRobotState(ros::Time::now(), 5.0);

    moveit_visual_tools::MoveItVisualTools visual_tools("base_link", rvt::RVIZ_MARKER_TOPIC,psm);

    moveit_msgs::MotionPlanResponse response;
    moveit_msgs::RobotTrajectory rtraj;

    {
        planning_scene_monitor::LockedPlanningSceneRW ps(psm);

        auto current_state = ps->getCurrentState();
        auto space = std::make_shared<ob::SE3StateSpace>();

        og::SimpleSetup ss(space);

        ob::RealVectorBounds bounds(3);
        bounds.setLow(-10.0);
        bounds.setHigh(10.0);
        space->setBounds(bounds);

        ob::ScopedState<ob::SE3StateSpace> start(space);

        start->setX(current_state.getJointPositions("world_joint")[0]);
        start->setY(current_state.getJointPositions("world_joint")[1]);
        start->setZ(current_state.getJointPositions("world_joint")[2]);
        start->rotation().x = current_state.getJointPositions("world_joint")[3];
        start->rotation().y = current_state.getJointPositions("world_joint")[4];
        start->rotation().z = current_state.getJointPositions("world_joint")[5];
        start->rotation().w = current_state.getJointPositions("world_joint")[6];

        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(-2.0, 2.0, 0.5);
        goal->rotation().setIdentity();

        ss.setStartAndGoalStates(start, goal);

        ss.setStateValidityChecker([&ps, current_state](const ob::State* st) {
            moveit::core::RobotState rs(current_state);

            double positions[] = {
                    st->as<ob::SE3StateSpace::StateType>()->getX(),
                    st->as<ob::SE3StateSpace::StateType>()->getY(),
                    st->as<ob::SE3StateSpace::StateType>()->getZ(),
                    st->as<ob::SE3StateSpace::StateType>()->rotation().x,
                    st->as<ob::SE3StateSpace::StateType>()->rotation().y,
                    st->as<ob::SE3StateSpace::StateType>()->rotation().z,
                    st->as<ob::SE3StateSpace::StateType>()->rotation().w
            };
            rs.setJointPositions("world_joint", positions);
            rs.update();

            return ps->isStateValid(rs);
        });

        ob::PlannerStatus solved = ss.solve(1.0);

        rtraj.multi_dof_joint_trajectory.joint_names.push_back("world_joint");

        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            // print the path to screen
            ss.simplifySolution();
            ss.getSolutionPath().print(std::cout);

            for (auto & st : ss.getSolutionPath().getStates()) {
                moveit::core::RobotState rs(current_state);

                trajectory_msgs::MultiDOFJointTrajectoryPoint mdjtp;
                geometry_msgs::Transform tf;

                tf.translation.x = st->as<ob::SE3StateSpace::StateType>()->getX();
                tf.translation.y = st->as<ob::SE3StateSpace::StateType>()->getY();
                tf.translation.z = st->as<ob::SE3StateSpace::StateType>()->getZ();

                tf.rotation.x = st->as<ob::SE3StateSpace::StateType>()->rotation().x;
                tf.rotation.y = st->as<ob::SE3StateSpace::StateType>()->rotation().y;
                tf.rotation.z = st->as<ob::SE3StateSpace::StateType>()->rotation().z;
                tf.rotation.w = st->as<ob::SE3StateSpace::StateType>()->rotation().w;

                mdjtp.transforms.push_back(tf);

                rtraj.multi_dof_joint_trajectory.points.push_back(mdjtp);
            }
        } else {
            ROS_ERROR("OMPL planner error: %s", solved.asString().c_str());
        }


//        std::vector<double> values;
//
//        const moveit::core::RobotModelConstPtr robot_model = ps->getRobotModel();
//        auto jmg = ps->getRobotModel()->getJointModelGroup("whole_body");
//
//        moveit::core::RobotState goal_state(robot_model);
//        std::vector<double> joint_values = {-2.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0};
//        goal_state.setJointGroupPositions(jmg, joint_values);
//        moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
//
//        planning_interface::MotionPlanRequest req;
//        req.group_name = PLANNING_GROUP;
//        req.goal_constraints.push_back(joint_goal);
//
//        moveit::core::RobotState start_state(ps->getCurrentState());
//
//        req.workspace_parameters.min_corner.x = -100.0;
//        req.workspace_parameters.min_corner.y = -100.0;
//        req.workspace_parameters.min_corner.z = 0.0;
//
//        req.workspace_parameters.max_corner.x = 100.0;
//        req.workspace_parameters.max_corner.y = 100.0;
//        req.workspace_parameters.max_corner.z = 100.0;
//
//        req.allowed_planning_time = 1.0;
//
//        planning_interface::MotionPlanResponse res;
//
//        planning_interface::PlannerManagerPtr planner_instance = initPlannerManager(nh, robot_model);
//
//        planning_interface::PlanningContextPtr context =
//                planner_instance->getPlanningContext(ps, req, res.error_code_);
//
//        context->solve(res);
//
//        if (res.error_code_.val != res.error_code_.SUCCESS) {
//            ROS_ERROR("Could not compute plan successfully");
//            return 0;
//        }
//
//
//        res.getMessage(response);
//
//        ROS_INFO("Found trajectory:");
//        for (auto &pt: response.trajectory.multi_dof_joint_trajectory.points) {
//            ROS_INFO(" - p: [%f,%f,%f] r: [%f,%f,%f,%f]",
//                     pt.transforms[0].translation.x, pt.transforms[0].translation.y,pt.transforms[0].translation.z,
//                     pt.transforms[0].rotation.x, pt.transforms[0].rotation.y, pt.transforms[0].rotation.z, pt.transforms[0].rotation.w);
//        }
    }
//


    const double *positions = psm->getStateMonitor()->getCurrentState()->getJointPositions("world_joint");

    auto gltf = psm->getStateMonitor()->getCurrentState()->getGlobalLinkTransform("base_link");

    ROS_INFO("[%f,%f,%f,%f,%f,%f,%f]", positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]);
    ROS_INFO("[%f,%f,%f,%f,%f,%f,%f]", gltf.translation().x(), gltf.translation().y(), gltf.translation().z(), gltf.rotation().data()[0], gltf.rotation().data()[1], gltf.rotation().data()[2], gltf.rotation().data()[3]);

    auto tem = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(psm->getRobotModel(), psm->getStateMonitor(), true);
    tem->push(rtraj, "drone_controller");

    tem->executeAndWait();

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

    return 0;
}

planning_interface::PlannerManagerPtr initPlannerManager(const ros::NodeHandle &nh, const moveit::core::RobotModelConstPtr& robot_model) {
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
    return planner_instance;
}
