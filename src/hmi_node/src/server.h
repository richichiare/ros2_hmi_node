#ifndef SERVER_H
#define SERVER_H
#include <fstream>
#include <vector>

#include "communication_primitives.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include "jsonprimitives.h"

class Server : public rclcpp::Node, public CommunicationPrimitives
{
public:
    Server();
    void hmi_send(void);
    void hmi_receive(void);
private:
    void setup_node_parameters(void);
    void init_server_socket(void);
    void bind_socket_to_address(void);
    void wait_for_data(void);
    void check_readable_socket(void);
    void publish_pose(void);
    bool finished_receiving(const std::vector<char> &recv_buf);
    void serve_request(void);

    /*Publisher*/
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
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
    std::vector<char> recv_buffer;
    ssize_t recv_bytes_received;
    size_t recv_offset;
    /*json*/
    json received_json;
    /*Constants*/
    const uint16_t SERVER_PORT = 5020;
};

#endif // SERVER_H
