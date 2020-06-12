#ifndef SERVER_H
#define SERVER_H
#include <fstream>

#include "communication_primitives.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/visibility_control.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

class Server : public rclcpp::Node, public CommunicationPrimitives
{
public:
    Server();
    void hmi_send(void);
    void hmi_receive(void);
private:
    /*Private class methods*/
    void setup_node_parameters(void);
    void init_server_socket(void);
    void bind_socket_to_address(void);
    void wait_for_data(void);
    void check_readable_socket(void);
    void publish_pose(void);

    /*Private members*/
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    fd_set read_fds, cpy_read_fds;
    struct sockaddr_in client_address;
    int client_socket_fd = 0;
    double resolution;
    std::vector<double> origin;
    std::string map_yaml_path;
    int ready_fd, max_fd;
    socklen_t client_len;
    Message m;
    ssize_t numBytesRecv;

    /*Constants*/
    const uint16_t SERVER_PORT = 5020;
};

#endif // SERVER_H
