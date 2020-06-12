#include "server.h"

Server::Server() : Node("hmi_node_server")
{
    setup_node_parameters();

    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    init_server_socket();

    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);

    for(;;){
        wait_for_data();
    }
    close(socket_fd);
}

void Server::hmi_send()
{

}

void Server::setup_node_parameters()
{
    this->declare_parameter("map_yaml_path");
    this->declare_parameter("resolution");
    this->declare_parameter("origin");
    this->get_parameter("map_yaml_path", map_yaml_path);

    YAML::Node param_file = YAML::LoadFile(map_yaml_path);
    resolution = param_file["resolution"].as<double>();
    origin = param_file["origin"].as<std::vector<double>>();

    std::ofstream fout(map_yaml_path.c_str());
    fout << param_file;
    fout.close();
}

void Server::init_server_socket()
{
    get_socket_fd_();

    int opt = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        RCLCPP_ERROR(this->get_logger(), "setsockopt");
        exit(EXIT_FAILURE);
    }

    bind_socket_to_address();

    if (listen(socket_fd, 1) < 0) {
        RCLCPP_ERROR(this->get_logger(), "listen");
        exit(EXIT_FAILURE);
    }
}

void Server::bind_socket_to_address()
{
    memset((void *)&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(SERVER_PORT);

    errno = 0;

    if (bind(socket_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind");
        RCLCPP_ERROR(this->get_logger(), "bind");
        exit(EXIT_FAILURE);
    }
}

void Server::wait_for_data()
{
    errno = 0;
    max_fd = std::max(socket_fd, client_socket_fd);
    cpy_read_fds = read_fds;

    printf("Waiting...\n");
    if((ready_fd = select(max_fd + 1/*FD_SETSIZE*/, &cpy_read_fds, NULL, NULL, NULL)) < 0){
        if(errno != EINTR){
            perror("select");
            exit(EXIT_FAILURE);
        }
    }
    check_readable_socket();
}

void Server::check_readable_socket()
{
    if(FD_ISSET(socket_fd, &cpy_read_fds)){
        memset((void *)&client_address, 0, sizeof(client_address));
        client_len = sizeof(client_address);
        client_socket_fd = accept(socket_fd, (struct sockaddr *)&client_address, &client_len); //accept connection from client
        FD_SET(client_socket_fd, &read_fds);
        RCLCPP_INFO(this->get_logger(), "Received connection");
    }else if(FD_ISSET(client_socket_fd, &cpy_read_fds)){
        RCLCPP_INFO(this->get_logger(), "Received data from client");
    }
    hmi_receive();
}

void Server::publish_pose()
{
    /*Do computation from pixel coordinates to map coordinates*/
    double final_x = (m.x/resolution) + origin[0];
    double final_y = ((704-m.y)/resolution) + origin[1];

    auto pose_msg = geometry_msgs::msg::PoseStamped();

    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";

    pose_msg.pose.orientation.w = 0;
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = 0;

    pose_msg.pose.position.x = final_x;
    pose_msg.pose.position.y = final_y;
    pose_msg.pose.position.z = 0;

    pose_publisher->publish(pose_msg);
}

void Server::hmi_receive(void){
    memset((void *)&m, 0, sizeof(Message));
    numBytesRecv = 0;
    do {
        errno = 0;
        numBytesRecv = recv(client_socket_fd, &m, sizeof(Message), 0);
        if (numBytesRecv < 0){
            RCLCPP_ERROR(this->get_logger(), "recv %s", strerror(errno));
            return;
        }else if(numBytesRecv == 0){
            RCLCPP_INFO(this->get_logger(), "Client closed connection!");
            FD_CLR(client_socket_fd, &read_fds);
            close(client_socket_fd);
            client_socket_fd = 0;
            return;
        }
    } while (numBytesRecv != sizeof(Message));
    RCLCPP_INFO(this->get_logger(), "Received %f %f", m.x, m.y);
    publish_pose();
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    //rclcpp::spinOnce(std::make_shared<Server>());
    Server *s = new Server();
    rclcpp::shutdown();
    return 0;
}
