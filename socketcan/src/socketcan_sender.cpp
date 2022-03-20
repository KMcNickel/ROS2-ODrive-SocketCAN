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
#include "can_interface/srv/can_frame.hpp"

#define MAX_DLC_LENGTH 8

using std::placeholders::_1;
using std::placeholders::_2;

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
                RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to create socket: %d", socketID);
                sysOK = false;
                return;
            }
            
            strcpy(ifr.ifr_name, interfaceName.c_str());
            
            if((err = ioctl(socketID, SIOCGIFINDEX, &ifr)) < 0)
            {
                RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to find interface: %d", err);
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
                RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to bind to socket: %d", err);
                sysOK = false;
                return;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bind to socket successful");

            service = this->create_service<can_interface::srv::CanFrame>("socketcan/sender", std::bind(&SocketCAN_Sender::serviceCallback, this, _1, _2));

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service ready");
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

        void serviceCallback (const std::shared_ptr<can_interface::srv::CanFrame::Request> request,
                            std::shared_ptr<can_interface::srv::CanFrame::Response> response)
        {
            can_frame frame;
            int err;

            if(request->frame.dlc > MAX_DLC_LENGTH)
            {
                response->status = response->STATUS_ERROR_INVALID_ID;
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                                "Message was sent with an invalid ID. IDE: %d - ID: %X", 
                                request->frame.is_extended_id, request->frame.can_id);
                    err = 1;
            }

            if((request->frame.is_extended_id && (request->frame.can_id & 0xE0000000))
                || (!request->frame.is_extended_id && (request->frame.can_id & 0xFFFFF800)))
                {
                    response->status = response->STATUS_ERROR_DLC_OUT_OF_RANGE;
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                                "Message was sent with an invalid DLC: %d", 
                                request->frame.dlc);
                    err = 1;
                }

            if(err != 0) return;      //We can list all warnings before exiting the function

            frame.can_id = request->frame.is_extended_id;
            frame.can_id = (frame.can_id << 1) | request->frame.is_remote_request;
            frame.can_id = (frame.can_id << 1) | request->frame.is_error;
            frame.can_id = (frame.can_id << 29) | request->frame.can_id;
            frame.can_dlc = request->frame.dlc;
            std::copy(std::begin(request->frame.data), std::end(request->frame.data), std::begin(frame.data));

            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending %d bytes to ID: %d", frame.can_dlc, frame.can_id);

            if((err = write(socketID, &frame, sizeof(struct can_frame))) != sizeof(struct can_frame))
            {
                if(err < 0)
                {
                    response->status = response->STATUS_ERROR_UNABLE_TO_SEND_DATA;
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to write data: %d", err);
                }
                else
                {
                    response->status = response->STATUS_ERROR_PARTIAL_FRAME_SENT;
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Could only write: %d bytes", err);
                }

                return;
            }
            response->status = response->STATUS_OK;
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Frame sent successfully");
        }
    
    private:
        bool sysOK;
        int socketID;
        struct ifreq ifr;
        struct sockaddr_can addr;
        rclcpp::Service<can_interface::srv::CanFrame>::SharedPtr service;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SocketCAN_Sender>());
    rclcpp::shutdown();
    return 0;
}