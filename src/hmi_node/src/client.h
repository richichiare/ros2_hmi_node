#ifndef CLIENT_H
#define CLIENT_H
#include "communication_primitives.hpp"
#include "rclcpp/rclcpp.hpp"

class Client : public rclcpp::Node, public CommunicationPrimitives
{
public:
    Client();
    void hmi_send();
    void hmi_receive();
private:
    void client_connect();
    void on_message_pose();

    const uint16_t PORT = 5000; //yaml
    const std::string server_address = "192.168.0.6";
    Message m;
    //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;

    // CommunicationPrimitives interface
};

#endif // CLIENT_H
