#ifndef ETHERNET_PRIMITIVES_H
#define ETHERNET_PRIMITIVES_H
#include "communication_primitives.hpp"

#include <stdio.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/select.h>
#include <errno.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/visibility_control.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#define SERVER_PORT 5020 

class EthernetPrimitives : public rclcpp::Node {
    public:
        EthernetPrimitives();
        ~EthernetPrimitives();
        void hmi_send(void);
        void hmi_receive(void);

    private:
        void get_socket_fd(void);
        void bind_socket_to_address(void);
        void init_server_socket(void);
        void wait_for_data(void);

        void publish_pose();
//        int open_socket_client(){

//            printf("Opening socket\n");
//            if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
//            {
//                printf("\n Socket creation error \n");
//                return -1;
//            }
        
//            serv_addr.sin_family = AF_INET;
//            serv_addr.sin_port = htons(SERVER_PORT);
            
//            // Convert IPv4 and IPv6 addresses from text to binary form
//            printf("Converting\n");
//            if(inet_pton(AF_INET, "172.15.0.104", &serv_addr.sin_addr)<=0)
//            {
//                printf("\nInvalid address/ Address not supported \n");
//                return -1;
//            }
//            printf("connecting\n");
//            if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
//            {
//                printf("\nConnection Failed \n");
//                return -1;
//            }
//            printf("Sending\n");
//            send(sock , hello , strlen(hello) , 0 );
//        }

        int server_socket_fd, client_socket_fd = 0;
        struct sockaddr_in server_address;
        Message m;
        struct sockaddr_in serv_addr, client_address;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
        fd_set read_fds, cpy_read_fds;
        double resolution;
        std::vector<double> origin;
        std::string map_yaml_path;
};
#endif
