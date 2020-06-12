#include "client.h"

Client::Client() : Node("hmi_node_client")
{
    get_socket_fd_();
    memset(&address, 0, sizeof(address));

    // Filling server information
    address.sin_family = AF_INET;
    address.sin_port = htons(PORT);
    address.sin_addr.s_addr = inet_addr(server_address.c_str());
    client_connect();
    on_message_pose();
    //pose_sub = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&Client::on_message_pose, this, _1));                                                                       cb_grp2_);
}

void Client::client_connect()
{
    while (connect(socket_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Connection failed");
        sleep(3); //Waiting before re-trying
    }
    RCLCPP_INFO(this->get_logger(), "Connected");
}

void Client::on_message_pose(/*const std_msgs::msg::String::SharedPtr msg*/) {
    //From pose to pixel coordinates
    m.x = (float) (rand()) / ((float) (RAND_MAX/759));
    m.y = (float) (rand()) / ((float) (RAND_MAX/704));
    hmi_send();
}

void Client::hmi_send() {
    short time = 0;
    sleep(4);
    while(++time){
        m.x += 10;
        send(socket_fd , &m , sizeof (Message), 0);
        RCLCPP_WARN(this->get_logger(), "Sending %f %f", m.x, m.y);
        sleep(1);
        if(time == 10)
            break;
    }
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
