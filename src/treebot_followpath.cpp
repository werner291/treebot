#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <algorithm>

#include <queue>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

int main(int argc, char **argv) {

    // Initialize the ROS node.
    ros::init(argc, argv, "treebot_path_follower", 0);

    // Get a reference to the current handle
    ros::NodeHandle nh;

    ros::Time start_time;
    moveit_msgs::RobotTrajectory traj;

    auto subscription = nh.subscribe<moveit_msgs::RobotTrajectory>("/best_trajectory", 10,
                                                                   [&](const moveit_msgs::RobotTrajectoryConstPtr &new_traj) {
                                                                       if (traj.multi_dof_joint_trajectory.points.empty()) start_time = ros::Time::now();
                                                                       traj = *new_traj;
                                                                       ROS_INFO("Trajectory update received.");
                                                                   }, nullptr);

    auto quad_target = nh.advertise<geometry_msgs::PoseStamped>("/quad_target", 10);

    // Initialize TF. We need the TransformListener to remain in scope while we use TF.
    tf2_ros::Buffer tfBuf(ros::Duration(10.0), true);
    tf2_ros::TransformListener tfList(tfBuf);

    ros::Rate rate(10.0);

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";

    while (nh.ok()) {
        try {
            ros::Duration time_from_start = ros::Time::now() - start_time;
            geometry_msgs::TransformStamped transformStamped = tfBuf.lookupTransform("map", "base_link", ros::Time(0));

            auto &points = traj.multi_dof_joint_trajectory.points;

            size_t segment_idx = 0;
            while (segment_idx + 1 < points.size() && points[segment_idx + 1].time_from_start < time_from_start) {
                segment_idx += 1;
            }

            if (segment_idx + 1 < points.size()) {

                auto point_before = points[segment_idx];
                auto point_after = points[segment_idx + 1];

                double t = (time_from_start.toSec() - point_before.time_from_start.toSec()) /
                           (point_after.time_from_start.toSec() - point_before.time_from_start.toSec());

                assert(0 <= t && t <= 1.0);

                msg.pose.position.x =
                        static_cast<float>(point_before.transforms[0].translation.x * (1.0 - t) +
                                           point_after.transforms[0].translation.x * t);
                msg.pose.position.y =
                        static_cast<float>(point_before.transforms[0].translation.y * (1.0 - t) +
                                           point_after.transforms[0].translation.y * t);

                msg.pose.position.z =
                        static_cast<float>(point_before.transforms[0].translation.z * (1.0 - t) +
                                           point_after.transforms[0].translation.z * t);

                Eigen::Quaterniond rot_before(point_before.transforms[0].rotation.w,
                                              point_before.transforms[0].rotation.x,
                                              point_before.transforms[0].rotation.y,
                                              point_before.transforms[0].rotation.z);
                Eigen::Quaterniond rot_after(point_after.transforms[0].rotation.w, point_after.transforms[0].rotation.x,
                                             point_after.transforms[0].rotation.y,
                                             point_after.transforms[0].rotation.z);

                auto rot_interp = rot_before.slerp(t, rot_after);

                msg.pose.orientation.x = static_cast<float>(rot_interp.x());
                msg.pose.orientation.y = static_cast<float>(rot_interp.y());
                msg.pose.orientation.z = static_cast<float>(rot_interp.z());
                msg.pose.orientation.w = static_cast<float>(rot_interp.w());

            } else {
                msg.pose.position.x = transformStamped.transform.translation.x;
                msg.pose.position.y = transformStamped.transform.translation.y;
                msg.pose.position.z = transformStamped.transform.translation.z;
                msg.pose.orientation.x = transformStamped.transform.rotation.x;
                msg.pose.orientation.y = transformStamped.transform.rotation.y;
                msg.pose.orientation.z = transformStamped.transform.rotation.z;
                msg.pose.orientation.w = transformStamped.transform.rotation.w;
            }

            msg.header.seq += 1;
            msg.header.stamp = ros::Time::now();
            quad_target.publish(msg);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ros::spinOnce();
    }

    return 0;
}