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
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "rclcpp/rclcpp.hpp"
#include "can_interface/msg/can_frame.hpp"

class SocketCAN : public rclcpp::Node
{
    public:
        SocketCAN(std::string interfaceName)
        : Node("SocketCAN")
        {
            sysOK = true;

            int err;

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

            publisher = this->create_publisher<can_interface::msg::CanFrame>("canRx", 50);

            while(rclcpp::ok())
                receiveData();
        }

        void receiveData()
        {
            struct can_frame frame;
            int err;
            auto message = can_interface::msg::CanFrame();

            if((err = read(socketID, &frame, sizeof(struct can_frame))) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to read data: %d", err);
                return;
            }

            message.can_id = frame.can_id & 0x1FFFFFFF;             //Get just the 11 or 29 bit ID
            message.is_error = frame.can_id & 0x20000000;           //Bit 29
            message.is_remote_request = frame.can_id & 0x40000000;  //Bit 30
            message.is_extended_id = frame.can_id & 0x80000000;     //Bit 31
            message.dlc = frame.can_dlc;
            memcpy(frame.data, &message.data, sizeof(frame.data));

            publisher->publish(message);
        }

        ~SocketCAN()
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
        bool sysOK;
        int socketID;
        struct ifreq ifr;
        struct sockaddr_can addr;
        rclcpp::Publisher<can_interface::msg::CanFrame>::SharedPtr publisher;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SocketCAN>("can0"));
    rclcpp::shutdown();
    return 0;
}