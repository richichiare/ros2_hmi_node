#include "server.h"

Server::Server() : Node("hmi_node_server") {
    recv_buffer.reserve(256);

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

void Server::setup_node_parameters() {

    this->declare_parameter("map_yaml_path");
    this->declare_parameter("path_to_rooms");
    this->declare_parameter("resolution");
    this->declare_parameter("origin");
    this->get_parameter("map_yaml_path", map_yaml_path);
    this->get_parameter("path_to_rooms", path_to_rooms);

    YAML::Node param_file = YAML::LoadFile(map_yaml_path);
    resolution = param_file["resolution"].as<double>();
    origin = param_file["origin"].as<std::vector<double>>();

    std::ofstream fout(map_yaml_path.c_str());
    fout << param_file;
    fout.close();

    YAML::Node rooms = YAML::LoadFile(path_to_rooms);
    for(YAML::const_iterator it = rooms.begin(); it != rooms.end(); ++it) {
        room_position.insert(std::pair<std::string, std::vector<float>>(it->first.as<std::string>(), it->second.as<std::vector<float>>()));
    }

    std::ofstream fout1(path_to_rooms.c_str());
    fout1 << param_file;
    fout1.close();
}

void Server::init_server_socket() {

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

void Server::bind_socket_to_address() {

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

void Server::wait_for_data() {

    errno = 0;
    max_fd = std::max(socket_fd, client_socket_fd);
    cpy_read_fds = read_fds;

    printf("Waiting...\n");
    if((ready_fd = select(max_fd + 1/*FD_SETSIZE*/, &cpy_read_fds, NULL, NULL, NULL)) < 0){
        if(errno != EINTR){
            perror("select");
            exit(EXIT_FAILURE);
        }
        perror("Attention!");
    }
    check_readable_socket();
}

void Server::check_readable_socket() {
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
    serve_request();
}

bool Server::finished_receiving(const std::vector<char> &recv_buf) {

    size_t index = recv_buf.size() - 1;
    if(recv_buf[index - 2] == '\\' &&
       recv_buf[index - 1] == '#' &&
       recv_buf[index - 1] == '$'){
        return true;
    }
    return false;
}

void Server::serve_request() {

    short intent_code = received_json["intent_code"];

    switch(intent_code){
        case 10: //Serve init request
            //TODO
            break;
        case 200: //Serve motion stop request
            //TODO
            break;
        case 100: //Serve motion move request
            publish_pose();
            break;
        case 300: //Serve status request
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Received unknown intent_code, ignoring");
    }
}

void Server::publish_pose() {
    /*Get coordinates from HMI*/
    float x_coord = (float) (received_json["data"]["location"])[0];
    float y_coord = (float) (received_json["data"]["location"])[1];

    /*Do computation from pixel coordinates to map coordinates*/
    double final_x = (x_coord * resolution) + origin[0];
    double final_y = ((704 - y_coord) * resolution) + origin[1];

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

void Server::hmi_receive(void) {
    recv_bytes_received = 0;
    recv_offset = 0;
    do{
        errno = 0;
        recv_bytes_received = recv(client_socket_fd, recv_buffer.data() + recv_offset,
                                   recv_buffer.size() - recv_offset, 0);
        if (recv_bytes_received < 0) { //Error occurred
            if (errno == EINTR) { //Interrupted by signal, try again
                continue;
            } else { //Interrupted for some reason
                perror("recv");
            }
        } else if (recv_bytes_received == 0) { //No more data available
            if (recv_offset == 0) { //Client closed connection
                FD_CLR(client_socket_fd, &read_fds);
                close(client_socket_fd);
                client_socket_fd = 0;
                return;
            } else { //Connection was closed while sending data...
                perror("recv");
                return;
            }
        } else if (finished_receiving(recv_buffer)) { //Message is complete
            recv_buffer.erase(recv_buffer.end() - 3, recv_buffer.end());
            received_json.clear();
            received_json = json::parse(std::string(recv_buffer.begin(), recv_buffer.end()));
            break;
        } else { //Message still not complete, iterate more
            recv_offset += recv_bytes_received;
            //buf.resize(std::max(buf.size(), 2 * offset)); // double available memory
        }
    } while(true);
}

int main(int argc, char **argv) {
    exit(-1);
    rclcpp::init(argc, argv);
    //rclcpp::spinOnce(std::make_shared<Server>());
    Server *s = new Server();
    rclcpp::shutdown();
    return 0;
}
