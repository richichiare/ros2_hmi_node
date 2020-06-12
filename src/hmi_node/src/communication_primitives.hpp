#ifndef COMMUNICATION_PRIMITIVES_H
#define COMMUNICATION_PRIMITIVES_H
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

struct message_t{
    float x, y;
};
typedef message_t Message;

class CommunicationPrimitives{
public:
    virtual void hmi_send(void) = 0;
    virtual void hmi_receive(void) = 0;
protected:
    void get_socket_fd_()
    {
        if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("socket");
            exit(EXIT_FAILURE);
        }
    };
    int socket_fd;
    struct sockaddr_in address;
};
#endif
