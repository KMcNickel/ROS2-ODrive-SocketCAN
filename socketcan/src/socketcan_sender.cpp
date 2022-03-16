#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "can_interface/msg/can_frame.hpp"

using std::placeholders::_1;

class SocketCAN_Sender : public rclcpp::Node
{
    public:
        SocketCAN_Sender()
        : Node("sender")
        {
            sysOK = true;
            int err;
            std::string interfaceName;

            this->declare_parameter<std::string>("interface_name", "can0");

            this->get_parameter("interface_name", interfaceName);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempting to connect on: %s", interfaceName.c_str());

            if ((socketID = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to create socket: %d", socketID);
                sysOK = false;
                return;
            }
            
            strcpy(ifr.ifr_name, interfaceName.c_str());
            
            if((err = ioctl(socketID, SIOCGIFINDEX, &ifr)) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to find interface: %d", err);
                sysOK = false;
                return;
            }

            if(!(ifr.ifr_flags & IFF_UP))
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Selected interface '%s' is not UP", interfaceName.c_str());

            memset(&addr, 0, sizeof(addr));
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if((err = bind(socketID, (struct sockaddr *)&addr, sizeof(addr))) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to bind to socket: %d", err);
                sysOK = false;
                return;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bind to socket successful");

            subscription = this->create_subscription<can_interface::msg::CanFrame>(
                "socketcan/sender/data", 10, std::bind(&SocketCAN_Sender::dataCallback, this, _1));
        }

        ~SocketCAN_Sender()
        {
            int err;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down");

            if((err = close(socketID)) < 0)
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to close socket: %d", err);
            else
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Socket closed");
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shut down complete. Goodbye!");
        }

    private:
        void dataCallback(const can_interface::msg::CanFrame & message) const
        {
            struct can_frame frame;
            int err;

            frame.can_id = message.is_extended_id;
            frame.can_id = (frame.can_id << 1) | message.is_remote_request;
            frame.can_id = (frame.can_id << 1) | message.is_error;
            frame.can_id = (frame.can_id << 29) | message.can_id;
            frame.can_dlc = message.dlc;
            std::copy(std::begin(message.data), std::end(message.data), std::begin(frame.data));

            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending %d bytes to ID: %d", frame.can_dlc, frame.can_id);

            if((err = write(socketID, &frame, sizeof(struct can_frame))) != sizeof(struct can_frame))
            {
                if(err < 0)
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to write data: %d", err);
                else
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Could only write: %d bytes", err);

                return;
            }
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Frame sent successfully");
        }
    
    private:
        bool sysOK;
        int socketID;
        struct ifreq ifr;
        struct sockaddr_can addr;
        rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr subscription;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SocketCAN_Sender>());
    rclcpp::shutdown();
    return 0;
}