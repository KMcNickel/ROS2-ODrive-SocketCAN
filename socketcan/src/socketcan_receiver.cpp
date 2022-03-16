#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "can_interface/msg/can_frame.hpp"

class SocketCAN_Sender : public rclcpp::Node
{
    public:
        SocketCAN_Sender()
        : Node("receiver")
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

            publisher = this->create_publisher<can_interface::msg::CanFrame>("socketcan/receiver/data", 50);

            pollDesc.fd = socketID;
            pollDesc.events = POLLIN;

            while(rclcpp::ok())
                receiveData();
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
        void receiveData()
        {
            struct can_frame frame;
            int err;
            auto message = can_interface::msg::CanFrame();
            int event;

            event = poll(&pollDesc, 1, 100);

            if(event < 0 && rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to poll socket: %d", event);
                return;
            }
            if(event > 0)
            {
                if((err = read(socketID, &frame, sizeof(struct can_frame))) < 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to read data: %d", err);
                    return;
                }

                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Received %d bytes on ID: %d", frame.can_dlc, frame.can_id);

                message.can_id = frame.can_id & 0x1FFFFFFF;             //Get just the 11 or 29 bit ID
                message.is_error = frame.can_id & 0x20000000;           //Bit 29
                message.is_remote_request = frame.can_id & 0x40000000;  //Bit 30
                message.is_extended_id = frame.can_id & 0x80000000;     //Bit 31
                message.dlc = frame.can_dlc;
                std::copy(std::begin(frame.data), std::end(frame.data), std::begin(message.data));

                publisher->publish(message);
            }
        }
    
        bool sysOK;
        int socketID;
        struct ifreq ifr;
        struct sockaddr_can addr;
        rclcpp::Publisher<can_interface::msg::CanFrame>::SharedPtr publisher;
        struct pollfd pollDesc;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SocketCAN_Sender>());
    rclcpp::shutdown();
    return 0;
}