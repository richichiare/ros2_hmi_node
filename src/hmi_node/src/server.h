#ifndef SERVER_H
#define SERVER_H
#include <fstream>
#include <vector>
#include <thread>

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

#include "jsonprimitives.h"
#include "statusinformation.h"

class Server : public rclcpp::Node, public CommunicationPrimitives
{
public:
    Server();
    void hmi_send(std::string message);
    bool hmi_receive(void);
private:
    void setup_node_parameters(void);
    void init_transform();
    void start_server_side();
    void init_server_socket(void);
    void bind_socket_to_address(void);
    void wait_for_request(void);
    void wait_for_data(void);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
    void check_readable_socket(void);
    void publish_pose(void);
    void send_feedback(bool feedback);
    void send_status(void);
    void send_init_response(void);
    bool finished_receiving(std::vector<char> *recv_buf, const ssize_t &bytes_received);
    void serve_request(void);

    /*Publisher*/
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    /*Subscriber*/
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    /*tf2 transform*/
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> readPose_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> transformedPose_;
    /*Keeping an object holding status info*/
    StatusInfo *status;
    /*select()*/
    fd_set read_fds, cpy_read_fds;
    int ready_fd, max_fd;
    /*ros2 parameters*/
    double resolution;
    std::vector<double> origin;
    std::string map_yaml_path, path_to_rooms;
    std::map<std::string, std::vector<float>> room_position;
    /*Client info*/
    struct sockaddr_in client_address;
    socklen_t client_len;
    int client_socket_fd = 0;
    /*recv()*/
    std::vector<char> *recv_buffer;
    ssize_t recv_bytes_received;
    size_t recv_offset;
    /*json*/
    json received_json;
    /*Constants*/
    const uint16_t SERVER_PORT = 5020;
    const std::string ID = "2";
    const std::string VERSION = "2.1";
};

#endif // SERVER_H
