#include "client.h"

Client::Client() : Node("hmi_node_client") {

    //connect_to_hmi_interface();
    setup_node_parameters();
    init_transform();

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom",  10, std::bind(&Client::odom_callback, this, std::placeholders::_1));
}

void Client::client_connect() {
    while (connect(socket_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Connection failed, retrying in 3 seconds...");
        sleep(3); //Waiting before re-trying
    }
    RCLCPP_INFO(this->get_logger(), "Connected");
}

//void Client::on_message_pose(/*const std_msgs::msg::String::SharedPtr msg*/) {
//    //From pose to pixel coordinates
//    m.x = (float) (rand()) / ((float) (RAND_MAX/759));
//    m.y = (float) (rand()) / ((float) (RAND_MAX/704));
//    hmi_send();
//}

void Client::hmi_send() {
    //send(socket_fd , &m , sizeof (Message), 0);
    RCLCPP_INFO(this->get_logger(), "Sending...");
}

void Client::init_transform() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp::Node::get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(rclcpp::Node::get_node_base_interface(), rclcpp::Node::get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    readPose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    transformedPose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
}

void Client::connect_to_hmi_interface() {

    get_socket_fd_();

    memset(&address, 0, sizeof(address));

    //HMI information
    address.sin_family = AF_INET;
    address.sin_port = htons(PORT);
    address.sin_addr.s_addr = inet_addr(server_address.c_str());

    client_connect();
}

void Client::odom_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg) {

    readPose_->pose = pose_msg->pose.pose;
    readPose_->pose.orientation.x = 0;
    readPose_->pose.orientation.y = 0;
    readPose_->pose.orientation.z = 0;
    readPose_->pose.orientation.w = 1;
    readPose_->header.frame_id = pose_msg->header.frame_id;
    try{
        tf_buffer_->transform(*readPose_, *transformedPose_, "map");
        map_x = transformedPose_->pose.position.x;
        map_y = transformedPose_->pose.position.y;

        map2pixel_coordinates();

        hmi_send();
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Tranform from odom to map failed, %s", ex.what());
    }
}

void Client::map2pixel_coordinates() {
    m.x = (map_x - origin[0])/resolution;
    m.y = -(((map_y - origin[1])/resolution)-704);
}

void Client::setup_node_parameters() {
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

void Client::hmi_receive()
{

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Client>());
    rclcpp::shutdown();
    return 0;
}
