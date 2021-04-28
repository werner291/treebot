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
#include <moveit/robot_state/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include <utility>

static const std::string PLANNING_GROUP = "whole_body";

namespace ob = ompl::base;
namespace og = ompl::geometric;

planning_interface::PlannerManagerPtr initPlannerManager(const ros::NodeHandle &nh, const moveit::core::RobotModelConstPtr& robot_model);

trajectory_msgs::MultiDOFJointTrajectoryPoint stateToTrajectoryPoint(ob::State *st);

namespace rvt = rviz_visual_tools;

class LookForwardValidator : public ob::MotionValidator {

public:
    LookForwardValidator(const ompl::base::SpaceInformationPtr &si,
                         std::shared_ptr<ob::DiscreteMotionValidator> super) : MotionValidator(si), super_(std::move(super)) {}

private:
    bool checkMotion(const ob::State *s1, const ob::State *s2) const override {



        auto ss1 = s1->as<ob::SE3StateSpace::StateType>();
        auto ss2 = s2->as<ob::SE3StateSpace::StateType>();

        Eigen::Vector3d forward_local(0.0,1.0,0.0);

        Eigen::Vector3d delta_linear(ss2->getX() - ss1->getX(), ss2->getY() - ss1->getY(), ss2->getZ() - ss1->getZ());
        double distance = delta_linear.norm();

        if (distance > std::numeric_limits<double>::epsilon()) {
            Eigen::Quaterniond rot1(ss1->rotation().w, ss1->rotation().x, ss1->rotation().y, ss1->rotation().z);
            auto fwd_1 = rot1 * forward_local;
            Eigen::Quaterniond rot2(ss2->rotation().w, ss2->rotation().x, ss2->rotation().y, ss2->rotation().z);
            auto fwd_2 = rot2 * forward_local;
            return super_->checkMotion(s1,s2) && (fwd_1.dot(delta_linear) / distance > 0.8) && (fwd_2.dot(delta_linear) / distance > 0.5);
        } else {
            return super_->checkMotion(s1,s2);
        }
    }

    bool checkMotion(const ob::State *s1, const ob::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override {
        ROS_ERROR("Direction checking not implemented for checkMotion with lastValid");
        return super_->checkMotion(s1,s2,lastValid);
    }

    std::shared_ptr<ob::DiscreteMotionValidator> super_;


};

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

    moveit_visual_tools::MoveItVisualTools visual_tools("map", "markers_viz",psm);
    visual_tools.deleteAllMarkers();

    visual_tools.trigger();

    moveit_msgs::MotionPlanResponse response;
    moveit_msgs::RobotTrajectory rtraj;

    {
        planning_scene_monitor::LockedPlanningSceneRW ps(psm);

        auto current_state = ps->getCurrentState();
        auto space = std::make_shared<ob::SE3StateSpace>();


        ob::RealVectorBounds bounds(3);
        bounds.setLow(-5.0);
        bounds.setHigh(5.0);
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


        auto si(std::make_shared<ob::SpaceInformation>(space));

        auto dmv = std::make_shared<ob::DiscreteMotionValidator>(si);
        auto mv(std::make_shared<LookForwardValidator>(si, dmv));
        si->setMotionValidator(std::static_pointer_cast<ob::MotionValidator>(mv));

        si->setStateValidityChecker([&ps, current_state](const ob::State* st) {
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

            if (positions[2] < 0.0) {
                return false;
            }

//            ROS_INFO("[%f,%f,%f]", positions[0],positions[1], positions[2]);

            rs.setJointPositions("world_joint", positions);
            rs.update();

            return ps->isStateValid(rs);
        });
        si->setup();

        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal, 0.5);

//        auto planner(std::make_shared<og::BITstar>(si));
        auto planner(std::make_shared<og::FMT>(si));
        planner->setProblemDefinition(pdef);

        ob::PlannerStatus solved = planner->ob::Planner::solve(60.0);

        rtraj.multi_dof_joint_trajectory.joint_names.push_back("world_joint");

        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            // print the path to screen
            pdef->getSolutionPath()->print(std::cout);

            for (auto & st : pdef->getSolutionPath()->as<og::PathGeometric>()->getStates()) {

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


        moveit_msgs::DisplayTrajectory dt;
        dt.trajectory.push_back(rtraj);

        moveit_msgs::RobotState robot_state_msg;
        moveit::core::robotStateToRobotStateMsg(current_state, robot_state_msg);

        moveit_msgs::DisplayTrajectory display_trajectory_msg;
        display_trajectory_msg.model_id = psm->getRobotModel()->getName();
        display_trajectory_msg.trajectory.resize(1);
        display_trajectory_msg.trajectory[0] = rtraj;
        display_trajectory_msg.trajectory_start = robot_state_msg;

        visual_tools.publishTrajectoryPath(display_trajectory_msg);
        visual_tools.trigger();
    }

    auto tem = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(psm->getRobotModel(), psm->getStateMonitor(), true);
    tem->push(rtraj, "drone_controller");

    tem->executeAndWait();
//
    ros::waitForShutdown();

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

trajectory_msgs::MultiDOFJointTrajectoryPoint stateToTrajectoryPoint(ob::State *st) {
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
    return mdjtp;
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
