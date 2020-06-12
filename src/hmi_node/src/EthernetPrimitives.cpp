#include "ethernet_primitives.hpp"
#include <fstream>

EthernetPrimitives::EthernetPrimitives() : Node("hmi_node_server") {
    //Create publisher for wheelchair pose
    FD_ZERO(&read_fds);
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

    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    init_server_socket();
    for(;;){
        wait_for_data();
    }
    close(server_socket_fd);
}

EthernetPrimitives::~EthernetPrimitives() {
    RCLCPP_WARN(this->get_logger(), "Destructor");
}


void EthernetPrimitives::init_server_socket(void){
    //Retrieving an fd for server socket
    get_socket_fd();
    //Binding the socket to a specific address
    bind_socket_to_address();
    if (listen(server_socket_fd, 1) < 0) {
        RCLCPP_ERROR(this->get_logger(), "listen");
        exit(EXIT_FAILURE);
    }
    FD_SET(server_socket_fd, &read_fds);
}

void EthernetPrimitives::get_socket_fd(void){
    if ((server_socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        RCLCPP_ERROR(this->get_logger(), "socket");
        exit(EXIT_FAILURE);
    }
    int opt = 1;
    if (setsockopt(server_socket_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        RCLCPP_ERROR(this->get_logger(), "setsockopt");
        exit(EXIT_FAILURE);
    }
}

void EthernetPrimitives::bind_socket_to_address(void){
    memset((void *)&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(SERVER_PORT);

    if (bind(server_socket_fd, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "bind");
        exit(EXIT_FAILURE);
    }
}

void EthernetPrimitives::wait_for_data(){
    int ready_fd;
    int max_fd;
    socklen_t client_len;
    errno = 0;
    max_fd = std::max(server_socket_fd, client_socket_fd);
    cpy_read_fds = read_fds;
    printf("Waiting...\n");
    if((ready_fd = select(max_fd + 1/*FD_SETSIZE*/, &cpy_read_fds, NULL, NULL, NULL)) < 0){
        if(errno != EINTR){
            perror("select");
            exit(EXIT_FAILURE);
        }
    }
    if(FD_ISSET(server_socket_fd, &cpy_read_fds)){
        memset((void *)&client_address, 0, sizeof(client_address));
        client_len = sizeof(client_address);
        client_socket_fd = accept(server_socket_fd, (struct sockaddr *)&client_address, &client_len); //accept connection from client
        FD_SET(client_socket_fd, &read_fds);
        printf("Received connection\n");
        hmi_receive();
    }
    if(FD_ISSET(client_socket_fd, &cpy_read_fds)){
        printf("Received data from client\n");
        hmi_receive();
    }
}

void EthernetPrimitives::hmi_receive(void){
    memset((void *)&m, 0, sizeof(Message));
    ssize_t numBytesRecv;
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

void EthernetPrimitives::publish_pose(){
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
