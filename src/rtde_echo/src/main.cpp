#include <netinet/in.h>
#include <ros/duration.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>

#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/spinner.h"
#include "rtde_echo/RtdeData.h"

// ==============================================

static bool is_big_endien;
static int  retry_count;
static int  buffer_size;

// ==============================================

struct RtdeMessage {
    char*    message;
    uint32_t len;
};

// Pre-cooked messages
static const RtdeMessage REQUEST_PROTOCOL_VERSION = {
    .message = (char*)"\x00\x05\x56\x00\x02",
    .len = 5
};
static const RtdeMessage CONTROL_PACKAGE_SETUP_OUTPUT = {
    .message = (char*)"\x00\x4e\x4f\x40\x3e\x00\x00\x00\x00\x00\x00""actual_robot_energy_consumed,actual_robot_braking_energy_dissipated",
    .len = 78
};
static const RtdeMessage CONTROL_PACKAGE_START = {
    .message = (char*)"\x00\x03\x53",
    .len = 3
};

// Pre-cooked response
static const char* PROTOCOL_VERSION_RESPONSE = "\x00\x04\x56\x01";
#define PROTOCOL_VERSION_RESPONSE_LENGTH 4
static const char* SETUP_OUTPUT_RESPONSE =
    "\x00\x11\x4f\x01"
    "DOUBLE,DOUBLE";
#define SETUP_OUTPUT_RESPONSE_LENGTH 17
static const char* PACKAGE_START_RESPONSE = "\x00\x04\x53\x01";
#define PACKAGE_START_RESPONSE_LENGTH 4
static const char* PACKAGE_HEADER = "\x00\x14\x55\x01";
#define PACKAGE_HEADER_LENGTH 4

// ==============================================

static int connect(uint32_t ip_addr, uint16_t port) {
    int                sockfd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in sock_address = {
        .sin_family = AF_INET,
        .sin_port = htons(30004),
        .sin_addr = {.s_addr = htonl(ip_addr)}
    };

    for (int attempt = 0; attempt < retry_count; attempt++) {
        if (connect(sockfd, (sockaddr*)&sock_address, sizeof(sock_address)) ==
            0) {
            return sockfd;
        }

        ROS_WARN("Cannot connect to the robot, retrying after 1 second.");
        ros::Duration(1, 0).sleep();
    }

    ROS_ERROR("Cannot connect to the robot after %d attempts.", retry_count);
    return -1;
}

static int8_t send_message(
    const uint32_t            sockfd,
    const struct RtdeMessage* message,
    unsigned char*            receive_buffer,
    const uint32_t            receive_buffer_len,
    const char*               expected_response,
    const uint32_t            expected_response_len
) {
    if (write(sockfd, message->message, message->len) == -1) {
        ROS_ERROR("Cannot send data");
        return -1;
    }

    uint32_t response_len = read(sockfd, receive_buffer, receive_buffer_len);

    if (response_len > expected_response_len) {
        ROS_ERROR("Unexpected response length");
        for (int i = 0; i < response_len; i++) {
            printf("0x%02hhX ", receive_buffer[i]);
        }
        printf("\n");
        return -2;
    }

    for (uint32_t index = 0; index < expected_response_len; index++) {
        if (receive_buffer[index] != expected_response[index]) {
            ROS_ERROR("Unexpected response");
            for (int i = 0; i < response_len; i++) {
                printf("0x%02hhX ", receive_buffer[i]);
            }
            printf("\n");
            return -3;
        }
    }

    return 0;
}

// Assuming that double is implemented using IEEE 754
static double ntohll(unsigned char* buffer) {
    double res;

    if (is_big_endien) {
        uint32_t slots[2];
        slots[0] = ntohl(*(uint32_t*)(buffer + 4));
        slots[1] = ntohl(*(uint32_t*)buffer);
        memcpy(&res, slots, 8);
    } else {
        // Do a memcpy to ensure alignment as the buffer may not be aligned
        memcpy(&res, buffer, 8);
    }

    return res;
}

static void
process_package(unsigned char* buffer, ros::Publisher& data_publisher) {
    for (int i = 0; i < PACKAGE_HEADER_LENGTH; i++) {
        if (PACKAGE_HEADER[i] != buffer[i]) {
            ROS_WARN("Malformed package received");
            return;
        }
    }

    // We got a package.
    rtde_echo::RtdeData data_package;
    data_package.energy_consumed = ntohll(buffer + 4);
    data_package.braking_energy_dissipated = ntohll(buffer + 12);

    data_publisher.publish(data_package);
}

int main(int argc, char** argv) {
    ROS_INFO("Starting rtde_echo node");

    ros::init(argc, argv, "rtde_echo");
    ros::NodeHandle node_handle;

    std::string     robot_ip_str;
    if (!node_handle.getParam(
            "/ur_hardware_interface/robot_ip", robot_ip_str
        )) {
        // fallback to the default IP from ursim
        robot_ip_str = "192.168.56.101";
    }
    if (!node_handle.getParam("/rtde_echo/retry_count", retry_count)) {
        retry_count = 3;
    }
    if (!node_handle.getParam("/rtde_echo/buffer_size", buffer_size)) {
        buffer_size = 1024;
    }

    // We assume that the IP address is somewhat well-formed
    uint32_t robot_ip = 0;
    {
        uint8_t ip1, ip2, ip3, ip4;
        if (sscanf(
                robot_ip_str.c_str(), "%hhu.%hhu.%hhu.%hhu", &ip1, &ip2, &ip3,
                &ip4
            ) != 4) {
            ROS_ERROR("Malformed IP address: %s", robot_ip_str.c_str());
            return -1;
        }

        robot_ip = (ip1 << 24) + (ip2 << 16) + (ip3 << 8) + ip4;
    }

    // 1 thread for the publisher
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Register publisher for the data received
    ros::Publisher rtde_data_publisher =
        node_handle.advertise<rtde_echo::RtdeData>(
            "/unity_bridge/rtde_data", 0
        );

    // [TODO]: make this mess handle error.
    is_big_endien = htonl(1) != 1;
    int sockfd = connect(robot_ip, 30004);
    if (sockfd == -1) {
        return -1;
    }

    ROS_INFO("Connected to the RTDE interface");

    // Communications!
    unsigned char* receive_buffer =
        (unsigned char*)malloc(sizeof(unsigned char) * buffer_size);

    ROS_INFO("Requesting protocol version 2");
    if (send_message(
            sockfd, &REQUEST_PROTOCOL_VERSION, receive_buffer, buffer_size,
            PROTOCOL_VERSION_RESPONSE, PROTOCOL_VERSION_RESPONSE_LENGTH
        ) != 0) {
        return -1;
    }

    ROS_INFO("Specifying interested dataset");
    if (send_message(
            sockfd, &CONTROL_PACKAGE_SETUP_OUTPUT, receive_buffer, buffer_size,
            SETUP_OUTPUT_RESPONSE, SETUP_OUTPUT_RESPONSE_LENGTH
        ) != 0) {
        return -1;
    }

    ROS_INFO("Starting data transfer");
    if (send_message(
            sockfd, &CONTROL_PACKAGE_START, receive_buffer, buffer_size,
            PACKAGE_START_RESPONSE, PACKAGE_START_RESPONSE_LENGTH
        ) != 0) {
        return -1;
    }

    struct pollfd fds = {
        .fd = sockfd, .events = POLLIN | POLLERR | POLLHUP | POLLNVAL
    };
    rtde_echo::RtdeData data_package;

    // Our package always is 20 bytes long
    uint32_t      read_len;
    uint32_t      read_ptr;
    unsigned char unfinished_package[20];
    uint32_t      unfinished_len = 0;

    while (!ros::isShuttingDown()) {
        read_ptr = 0;

        // Read from file descriptor
        if (poll(&fds, 1, 500) == 1) {
            switch (fds.revents) {
                case POLLIN: {
                    // the fd is ready to be read
                    read_len = read(sockfd, receive_buffer, buffer_size);
                    break;
                }
                case POLLHUP: {
                    ROS_ERROR("RTDE interface disconnected");
                    return -1;
                }
                default: {
                    ROS_ERROR("Unhandled error");
                    return -1;
                }
            }
        }

        // Process the buffer
        if (unfinished_len != 0) {
            // We have unfinished package, memcpy the remaining data over and
            // process that package
            memcpy(
                unfinished_package + unfinished_len, receive_buffer,
                20 - unfinished_len
            );
            process_package(unfinished_package, rtde_data_publisher);
            read_ptr = 20 - unfinished_len;
            unfinished_len = 0;
        }

        uint32_t loop_count = read_len / 20;
        unfinished_len = read_len % 20;

        for (uint32_t loop = 0; loop < loop_count; loop++) {
            process_package(receive_buffer + read_ptr, rtde_data_publisher);
            read_ptr += 20;
        }

        // memcpy half-received package if exist
        if (unfinished_len != 0) {
            memcpy(
                unfinished_package, receive_buffer + read_ptr, unfinished_len
            );
        }
    }

    return 0;
}