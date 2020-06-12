#include "rclcpp/rclcpp.hpp"
#include "ethernet_primitives.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    EthernetPrimitives *h = new EthernetPrimitives();
    rclcpp::shutdown();
    return 0;
}
