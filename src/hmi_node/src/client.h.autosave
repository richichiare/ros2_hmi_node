#ifndef CLIENT_H
#define CLIENT_H
#include <fstream>

#include "communication_primitives.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/buffer.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#define LOCALHOST

class Client : public rclcpp::Node, public CommunicationPrimitives
{
public:
    Client();
    void hmi_send();
    void hmi_receive();
private:
    void client_connect();
    void init_transform();
    void connect_to_hmi_interface();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
    void map2pixel_coordinates();
    void setup_node_parameters();

    const uint16_t PORT = 5000;
#ifndef LOCALHOST
    const std::string server_address = "192.168.0.6";
#else
    const std::string server_address = "127.0.0.1";
#endif
    double map_x, map_y;
    double resolution;
    std::vector<double> origin;
    std::string map_yaml_path;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<geometry_msgs::msg::PoseStamped> readPose_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> transformedPose_;
};

#endif // CLIENT_H
