//
// Created by werner on 11-06-21.
//

#include "RealtimeTrajectoryExecutor.h"

RealtimeTrajectoryExecutor::RealtimeTrajectoryExecutor(ros::NodeHandle &nh, tf2_ros::Buffer &tf) : nh(nh), tf(tf) {

}

void RealtimeTrajectoryExecutor::start() {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedValue"
    background_thread = std::thread([this] { run(); });
#pragma clang diagnostic pop
}

geometry_msgs::Pose transformToPose(const geometry_msgs::Transform& transform) {
    geometry_msgs::Pose pose;

    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;

    pose.orientation.x = transform.rotation.x;
    pose.orientation.y = transform.rotation.y;
    pose.orientation.z = transform.rotation.z;
    pose.orientation.w = transform.rotation.w;

    return pose;
}

void RealtimeTrajectoryExecutor::run() {

    ros::Rate rate(ros::Duration(1.0/20.0));

    auto quad_target = nh.advertise<geometry_msgs::PoseStamped>("/quad_target", 10);

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.seq = 0;

    while (nh.ok()) {

        try {
            geometry_msgs::TransformStamped transformStamped = tf.lookupTransform("map", "base_link", ros::Time(0));

            std::lock_guard<std::mutex> lock(mutex);

            ros::Time now = ros::Time::now();

            size_t segment_idx = 0;
            while (segment_idx + 1 < trajectory.size() && trajectory[segment_idx + 1].at_time < now) {
                segment_idx += 1;
            }

            if (segment_idx + 1 < trajectory.size()) {

                auto point_before = trajectory[segment_idx];
                auto point_after = trajectory[segment_idx + 1];

                double t = (now.toSec() - point_before.at_time.toSec()) /
                           (point_after.at_time.toSec() - point_before.at_time.toSec());

                assert(0 <= t && t <= 1.0);

                auto tf_before = point_before.state.getGlobalLinkTransform("base_link");
                auto tf_after = point_after.state.getGlobalLinkTransform("base_link");

                msg.pose.position.x =
                        static_cast<float>(tf_before.translation().x() * (1.0 - t) +
                                           tf_after.translation().x() * t);
                msg.pose.position.y =
                        static_cast<float>(tf_before.translation().y() * (1.0 - t) +
                                           tf_after.translation().y() * t);

                msg.pose.position.z =
                        static_cast<float>(tf_before.translation().z() * (1.0 - t) +
                                           tf_after.translation().z() * t);

                auto rot_interp = Eigen::Quaterniond(tf_before.rotation()).slerp(t, Eigen::Quaterniond(tf_after.rotation()));

                msg.pose.orientation.x = static_cast<float>(rot_interp.x());
                msg.pose.orientation.y = static_cast<float>(rot_interp.y());
                msg.pose.orientation.z = static_cast<float>(rot_interp.z());
                msg.pose.orientation.w = static_cast<float>(rot_interp.w());

            } else {
                msg.pose = transformToPose(transformStamped.transform);
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

        rate.sleep();
    }
}

void RealtimeTrajectoryExecutor::updateTrajectory(const std::vector<TrajectoryPoint> new_trajectory) {
    std::lock_guard<std::mutex> lock(mutex);

    trajectory = new_trajectory;
}


