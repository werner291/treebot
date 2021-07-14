#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <vector>
#include <sensor_msgs/Image.h>

const size_t RESOLUTION = 50;


int main(int argc, char **argv) {

    // Initialize the ROS node.
    ros::init(argc, argv, "treebot_controller", 0);

    // Get a reference to the current handle
    ros::NodeHandle nh;

    struct Cell {
        float amplitude;
        float phase;
        float frequency;
    };

    std::vector<Cell> occupancy(RESOLUTION * RESOLUTION);

    Eigen::AlignedBox2d bounds(Eigen::Vector2d(-5.0, -5.0), Eigen::Vector2d(5.0, 5.0));

    auto predicted = nh.advertise<sensor_msgs::Image>("/periodic_predicted", 100);

    // Subscribe to the /touch_targets topic, and update the targets when a message comes in.
    auto targets_sub = nh.subscribe<sensor_msgs::Image>(
            "/drone/front/depth", 100,
            [&](const sensor_msgs::ImageConstPtr& image_msg) {

                double t = image_msg->header.stamp.toSec();

                assert(image_msg->encoding == "32FC1");

                for (size_t cam_y = 0; cam_y < image_msg->height; cam_y++) {
                    for (size_t cam_x = 0; cam_x < image_msg->width; cam_x++) {

                        image_msg->



                    }
                }

                sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(*points_msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(*points_msg, "z");

                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                    
                    Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);

                    auto x_idx = (size_t) floor((double) RESOLUTION * (point.x() - bounds.min().x()) / (bounds.max().x() - bounds.min().x()));
                    auto y_idx = (size_t) floor((double) RESOLUTION * (point.y() - bounds.min().y()) / (bounds.max().y() - bounds.min().y()));
                    auto z_idx = (size_t) floor((double) RESOLUTION * (point.z() - bounds.min().z()) / (bounds.max().z() - bounds.min().z()));

                    // size_t is unsigned, so anything below lower bound rolls over and will be caught by the upper bound check
                    if (x_idx >= RESOLUTION || y_idx >= RESOLUTION || z_idx >= RESOLUTION) {
                        size_t index = x_idx + y_idx * RESOLUTION + z_idx * RESOLUTION * RESOLUTION;

                        Cell c = occupancy[index];

                        double predicted = c.amplitude * sin(c.frequency * t + c.phase);

                        double error = 1.0 - predicted;

                    }

                }
                
            });

    ros::spin();

}