#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <boost/graph/directed_graph.hpp>

#include <queue>
#include <memory>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "moveit_interaction.h"
#include "RealtimeTrajectoryExecutor.h"
#include <mutex>

namespace rvt = rviz_visual_tools;

class EigenVector3dHash {

    std::hash<double> hash;

public:
    size_t operator()(const Eigen::Vector3d &v) const {
        size_t h = hash(v.x());
        h = hash(v.y()) + h * 31;
        h = hash(v.z()) + h * 31;
        return h;
    }

};

typedef std::unordered_set<Eigen::Vector3d, EigenVector3dHash> TargetSet;

planning_scene::PlanningScenePtr
snapshotPlanningScene(const std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> &psm);

moveit::core::RobotState
floatingJointStateFromPositionANdRotation(const moveit::core::RobotState &node_state, Eigen::Quaterniond &next_rot,
                                          const Eigen::Vector3d &next_position);

moveit::core::RobotState
sampleNearbyState(std::default_random_engine &rng, std::uniform_real_distribution<double> &angles,
                  const moveit::core::RobotState &node_state);

geometry_msgs::Transform baseLinkTransform(const moveit::core::RobotState &state);

moveit_msgs::RobotTrajectory
StateVectorToTrajectory(std::vector<TrajectoryPoint> &trajectory, const ros::Time& start_time);

std::pair<double, Eigen::Vector3d>
costToGoToNearest(const TargetSet &targets, const Eigen::Vector3d &end_effector_pos) {
    double square_distance_to_nearest_target = std::numeric_limits<double>::infinity();
    Eigen::Vector3d closest_target;
    for (const Eigen::Vector3d &point : targets) {

        double sqr_dst = (point - end_effector_pos).squaredNorm();

        if (sqr_dst < square_distance_to_nearest_target) {
            square_distance_to_nearest_target = sqr_dst;
            closest_target = point;
        }
    }
    return std::make_pair(sqrt(square_distance_to_nearest_target), closest_target);
}

std::vector<Eigen::Vector3d> pointCloudToTargets(const sensor_msgs::PointCloud2ConstPtr &points_msg) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*points_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*points_msg, "z");

    std::vector<Eigen::Vector3d> targets;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        targets.emplace_back(*iter_x, *iter_y, *iter_z);
    }
    return targets;
}

double pathQuality(const std::vector<Eigen::Vector3d> &latest_targets,
                   const std::vector<TrajectoryPoint> &best_trajectory) {

    TargetSet remaining_targets;

    remaining_targets.insert(latest_targets.begin(), latest_targets.end());

    double path_quality = 0.0;

    size_t future = 0;

    for (const auto &state : best_trajectory) {
        future++;

        const moveit::core::LinkModel *endEffector = state.state.getLinkModel("end_effector");
        auto endEffectorXform = state.state.getGlobalLinkTransform(endEffector);
        Eigen::Vector3d endEffectorPos = endEffectorXform.translation();

        double targetDistance;
        Eigen::Vector3d nearestTarget;
        std::tie(targetDistance, nearestTarget) = costToGoToNearest(remaining_targets, endEffectorPos);

        double future_factor = tanh((double) future / 100.0);

        if (targetDistance < TARGET_REACH_THRESHOLD * (1.0 + future_factor * 15.0)) {

            remaining_targets.erase(nearestTarget);

            path_quality += 1.0 - future_factor;
        } else {
            path_quality -= 0.01;
        }
    }

    return path_quality;
}

int main(int argc, char **argv) {

    // Initialize the ROS node.
    ros::init(argc, argv, "treebot_controller", 0);

    // Get a reference to the current handle
    ros::NodeHandle nh;

    // Start ros messaging in the background
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Keep a list of all known targets, which may be updated by ROS, so we needf a mutex.
    std::mutex targets_mutex;
    std::vector<Eigen::Vector3d> latest_targets;

    // Subscribe to the /touch_targets topic, and update the targets when a message comes in.
    auto targets_sub = nh.subscribe<sensor_msgs::PointCloud2>("/touch_targets", 100, [&targets_mutex, &latest_targets](
            const sensor_msgs::PointCloud2ConstPtr &points_msg) {

        // Lock the mutex so we know we're the only ones changing the targets.
        const std::lock_guard<std::mutex> lock(targets_mutex);

        // Update the variable
        latest_targets = pointCloudToTargets(points_msg);
    });

    // Initialize TF. We need the TransformListener to remain in scope while we use TF.
    auto tfBuf = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0), true);
    auto tfList = std::make_shared<tf2_ros::TransformListener>(*tfBuf);

    // Get a PlanningSceneMonitor that will maintain an up-to-date view of the scene for us.
    auto psm = initPlanningSceneMonitor(tfBuf, 1.0);

    // Initialize interaction with RViz
    auto visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("map", "markers_viz", psm);
    visual_tools->deleteAllMarkers();
//
//    // The planner works by repeatedly trying to improve the current path, which will be updated frequently.
//    // We can't use a TrajectoryExecutionManager since it has too strong of a start/stop notion.
//    auto current_trajectory_pub = nh.advertise<moveit_msgs::RobotTrajectory>("current_best_trajectory", 10);

//    auto tem = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(psm->getRobotModel(),
//                                                                                          psm->getStateMonitor(), true);
//    tem->setAllowedStartTolerance(0.5);

    RealtimeTrajectoryExecutor follower(nh, *tfBuf);
    follower.start();

    // Initialize the RNG with the current time.
    std::default_random_engine rng(ros::Time::now().toNSec());

    ros::Time last_published;

    ros::Time start_time = ros::Time::now();

    std::vector<TrajectoryPoint> best_trajectory;

    // Place current state at the start of the trajectory
    best_trajectory.insert(best_trajectory.begin(), {
            snapshotPlanningScene(psm)->getCurrentState(), start_time
    });

    while (ros::ok()) {

        planning_scene::PlanningScenePtr ps = snapshotPlanningScene(psm);

        ros::Time current_time = ros::Time::now();

        const moveit::core::RobotState start_state = ps->getCurrentState();

        // If any states have become invalid, truncate down to that point.
        for (auto iter = best_trajectory.begin(); iter != best_trajectory.end(); ++iter) {
            if (!ps->isStateValid(iter->state)) {
                ROS_INFO("Trajectory invalidated.");
                best_trajectory.erase(iter, best_trajectory.end());
                break;
            }
        }

        // Erase any past states from current best trajectory
        while (best_trajectory.size() >= 2 && best_trajectory[1].at_time < current_time) {
            best_trajectory.erase(best_trajectory.begin());
        }



        // Allocate a candidate trajectory
        std::vector<TrajectoryPoint> candidate_trajectory;

        // We branch off from the current best trajectory from a point selected uniformly at random.
        size_t branch_point = (size_t) std::uniform_int_distribution(1, (int) best_trajectory.size())(rng);
        // We copy the shared parts into the candidate trajectory
        candidate_trajectory.insert(candidate_trajectory.end(), best_trajectory.begin(), best_trajectory.begin() + (long) branch_point);


        std::uniform_real_distribution<double> angles(-0.3, 0.3);
        std::uniform_real_distribution<double> zerotoOne(0.0, 1.0);

        for (size_t i = 0; i < branch_point; i++) {
            auto traj_pt = best_trajectory[i];

            candidate_trajectory.push_back(traj_pt);
        }

        for (size_t i = branch_point; i < 100; i++) {
            moveit::core::RobotState next_state = sampleNearbyState(rng, angles, candidate_trajectory.back().state);

            if (ps->isStateValid(next_state)) {
                double state_delta = candidate_trajectory.back().state.distance(next_state);
                candidate_trajectory.push_back({next_state, candidate_trajectory.back().at_time + ros::Duration(
                        state_delta * 0.2)});
            } else {
                break;
            }
        }

        {
            double best_quality, current_quality;
            {
                const std::lock_guard<std::mutex> lock(targets_mutex);

                best_quality = pathQuality(latest_targets, best_trajectory);
                current_quality = pathQuality(latest_targets, candidate_trajectory);
            }

            if (current_quality > best_quality) {

                ROS_INFO("New best found!");

                best_trajectory = candidate_trajectory;
            }
        }

        // I'll probably want to push this into a background thread.
        if (ros::Time::now() - last_published > ros::Duration(0.1)) {
            follower.updateTrajectory(best_trajectory);

            moveit_msgs::RobotTrajectory rtraj = StateVectorToTrajectory(best_trajectory, start_time);

            displayMultiDoFTrajectory(visual_tools, rtraj, ps->getCurrentState());

            last_published = ros::Time::now();
        }
    }

    ROS_INFO("\\033[32m Task completed, waiting for shutdown signal.");

    ros::waitForShutdown();

    return 0;
}

moveit_msgs::RobotTrajectory StateVectorToTrajectory(std::vector<TrajectoryPoint> &trajectory, const ros::Time& start_time) {
    moveit_msgs::RobotTrajectory rtraj;
    rtraj.multi_dof_joint_trajectory.joint_names.push_back("world_joint");


    for (const auto &state: trajectory) {
        geometry_msgs::Transform tf = baseLinkTransform(state.state);

        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt;
        trajpt.transforms.push_back(tf);
        trajpt.time_from_start = state.at_time - start_time;

        rtraj.multi_dof_joint_trajectory.points.push_back(trajpt);
    }
    return rtraj;
}

geometry_msgs::Transform baseLinkTransform(const moveit::core::RobotState &state) {
    geometry_msgs::Transform tf;

    const double *floating_joint_positions = state.getVariablePositions();

    tf.translation.x = floating_joint_positions[0];
    tf.translation.y = floating_joint_positions[1];
    tf.translation.z = floating_joint_positions[2];
    tf.rotation.x = floating_joint_positions[3];
    tf.rotation.y = floating_joint_positions[4];
    tf.rotation.z = floating_joint_positions[5];
    tf.rotation.w = floating_joint_positions[6];
    return tf;
}

moveit::core::RobotState
sampleNearbyState(std::default_random_engine &rng,
                  std::uniform_real_distribution<double> &angles,
                  const moveit::core::RobotState &node_state) {
    const double *floating_joint_positions = node_state.getVariablePositions();

    Eigen::Vector3d base_position(floating_joint_positions[0], floating_joint_positions[1],
                                  floating_joint_positions[2]);

    // Note: Eigen's quaternions are [w,x,y,z], but the floating joint has [x,y,z,w]
    Eigen::Quaterniond rot(
            floating_joint_positions[6], floating_joint_positions[3],
            floating_joint_positions[4], floating_joint_positions[5]);


    const double MAX_STEP = 0.3;

    std::uniform_real_distribution<double> distance_distribution(0.0, MAX_STEP);

    double forward_distance = distance_distribution(rng);

    double change_factor = 1.0 - forward_distance / MAX_STEP;

    std::uniform_real_distribution<double> vertical_distribution(-0.1 * change_factor, 0.1 * change_factor);

    std::uniform_real_distribution<double> angle_distribution(-0.5 * change_factor, 0.5 * change_factor);
    auto next_rot = rot * Eigen::AngleAxisd(angles(rng), Eigen::Vector3d::UnitZ());

    Eigen::Vector3d next_position =
            base_position + next_rot * Eigen::Vector3d(0.0, forward_distance, vertical_distribution(rng));

    moveit::core::RobotState next_state = floatingJointStateFromPositionANdRotation(node_state, next_rot,
                                                                                    next_position);
    next_state.update();
    return next_state;
}

moveit::core::RobotState
floatingJointStateFromPositionANdRotation(const moveit::core::RobotState &node_state, Eigen::Quaterniond &next_rot,
                                          const Eigen::Vector3d &next_position) {
    moveit::core::RobotState next_state(node_state);

    double next_joint_positions[7] = {
            next_position.x(),
            next_position.y(),
            next_position.z(),
            next_rot.x(),
            next_rot.y(),
            next_rot.z(),
            next_rot.w()
    };

    next_state.setVariablePositions(next_joint_positions);
    return next_state;
}