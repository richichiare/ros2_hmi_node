#include "server.h"

Server::Server() : Node("hmi_node_server") {

    /*Start server side which waits for requests from high level*/
    start_server_side();

    /*Read ros2 node params for resolution and origin of ros2 map*/
    setup_node_parameters();

    /*Init tf2 transform parameters for odom->map coordinates*/
    init_transform();

    status = new StatusInfo();

    /*Publisher for moving wheelchair*/
    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    /*Subscriber for wheelchair current location*/
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom",  10, std::bind(&Server::odom_callback, this, std::placeholders::_1));
}

void Server::start_server_side(){

    recv_buffer = new std::vector<char>(256);
    init_server_socket();
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);

    /*Launching thread which blocks on select()*/
    std::thread server(&Server::wait_for_request, this);
    server.detach();
}

void Server::setup_node_parameters() {

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

void Server::init_transform() {

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp::Node::get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(rclcpp::Node::get_node_base_interface(), rclcpp::Node::get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    readPose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    transformedPose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
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

void Server::wait_for_request() {

    for(;;){
        wait_for_data();
    }
    close(socket_fd);
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
    } else if(FD_ISSET(client_socket_fd, &cpy_read_fds)) {
        RCLCPP_INFO(this->get_logger(), "Received data from client");
        if(hmi_receive()){
            serve_request();
        }
    }
}

bool Server::finished_receiving(std::vector<char> *recv_buf, const ssize_t &bytes_received) {

    size_t index = bytes_received - 1;

    if(recv_buf->at(index - 2) == '\\' &&
       recv_buf->at(index - 1) == '#' &&
       recv_buf->at(index) == '$'){
        return true;
    }
    return false;
}

void Server::serve_request() {

    short intent_code = received_json["intent_code"];

    switch(intent_code){
        case 10: //Serve init request
            send_init_response();
            break;
        case 200: //Serve motion stop request
            //TODO
            break;
        case 100: //Serve motion move request
            if(1/*wheelchair_can_move*/){
                publish_pose();
                send_feedback(true);
            }else{
                send_feedback(false);
            }
            break;
        case 300: //Serve status request
            send_status();
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Received unknown intent_code, ignoring");
    }
}

void Server::odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg) {

    readPose_->pose = pose_msg->pose.pose;
    readPose_->pose.orientation.x = 0;
    readPose_->pose.orientation.y = 0;
    readPose_->pose.orientation.z = 0;
    readPose_->pose.orientation.w = 1;
    readPose_->header.frame_id = pose_msg->header.frame_id;
    try{
        tf_buffer_->transform(*readPose_, *transformedPose_, "map");
        status->setLocation(transformedPose_->pose.position.x, transformedPose_->pose.position.y);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Tranform from odom to map failed, %s", ex.what());
    }
}

void Server::publish_pose() {
    /*Get coordinates from HMI*/
    float x_coord = (float) (received_json["data"]["target"])[0];
    float y_coord = (float) (received_json["data"]["target"])[1];

    RCLCPP_INFO(this->get_logger(), "%f %f", x_coord, y_coord);

    /*Do computation from pixel coordinates to map coordinates*/
    double final_x = (x_coord * resolution) + origin[0];
    double final_y = ((704 - y_coord) * resolution) + origin[1];

    status->setTarget(final_x, final_y);

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

void Server::send_feedback(bool feedback) {
    json j_feedback;
    std::string j_feedback_string;
    short intent_code = received_json["intent_code"];
    std::string tag = received_json["tag"];

    build_feedback_resp_message(j_feedback, intent_code, tag, feedback);
    j_feedback_string = j_feedback.dump();
    append_delimiter(j_feedback_string);

    hmi_send(j_feedback_string);
}

void Server::send_status() {
    json j_status;
    std::string j_status_string;
    std::string tag = received_json["tag"];
//    float x_loc, y_loc, x_target, y_target;
//    status->getLocation(x_loc, y_loc);
//    status->getTarget(x_target, y_target);
    float x = 250.5f, y = 333.4f;

    build_status_resp_message(j_status, tag, x, y, x, x, (short)1, (short)33, true);
    j_status_string = j_status.dump();

    append_delimiter(j_status_string);

    hmi_send(j_status_string);
}


void Server::send_init_response(){
    json j_init_resp;
    std::string j_init_resp_string;
    std::string tag = received_json["tag"];
    build_init_resp_message(j_init_resp, tag, ID, VERSION);
    append_delimiter(j_init_resp_string);

    hmi_send(j_init_resp_string);
}

void Server::hmi_send(std::string message) {
    if(!client_socket_fd){
        RCLCPP_ERROR(this->get_logger(), "send(): bad client socket");
        return;
    }
    send(client_socket_fd , message.c_str(), message.length(), 0);
    RCLCPP_INFO(this->get_logger(), "send(): %s", message.c_str());
}

bool Server::hmi_receive(void) {

    memset(recv_buffer->data(), 0, recv_buffer->size());
    recv_bytes_received = 0;
    recv_offset = 0;
    do {
        errno = 0;
        recv_bytes_received = recv(client_socket_fd, recv_buffer->data() + recv_offset,
                                   recv_buffer->size() - recv_offset, 0);
        if (recv_bytes_received < 0) { //Error occurred
            if (errno == EINTR) { //Interrupted by signal, try again
                continue;
            } else { //Interrupted for some reason
                perror("recv()");
                return false;
            }
        } else if (recv_bytes_received == 0) { //No more data available
            if (recv_offset == 0) { //Client closed connection
                RCLCPP_INFO(this->get_logger(), "connection was closed by peer");
                FD_CLR(client_socket_fd, &read_fds);
                close(client_socket_fd);
                client_socket_fd = 0;
                return false;
            } else { //Connection was closed while sending data...
                perror("unexpected recv()");
                return false;
            }
        } else if (finished_receiving(recv_buffer, recv_bytes_received)) { //Message is complete
            RCLCPP_INFO(this->get_logger(), "Finish: %s", std::string(recv_buffer->begin(), recv_buffer->begin() + recv_bytes_received - 3).c_str());
            received_json.clear();
            received_json = json::parse(std::string(recv_buffer->begin(), recv_buffer->begin() + recv_bytes_received - 3));
            return true;
        } else { //Message still not complete, iterate more
            RCLCPP_INFO(this->get_logger(), "More: %s", std::string(recv_buffer->begin(), recv_buffer->end()).c_str());
            recv_offset += recv_bytes_received;
        }
    } while(true);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Server>());
    rclcpp::shutdown();
    return 0;
}
