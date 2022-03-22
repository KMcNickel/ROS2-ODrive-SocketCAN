#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <cstring>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "can_interface/srv/can_frame.hpp"

#define MAX_DLC_LENGTH 8
#define SOCKET_CLOSED_PROGRAMATICALLY -10
#define CAN_ID_LARGER_THAN_29_BIT_MASK 0xE0000000
#define CAN_ID_LARGER_THAN_11_BIT_MASK 0xFFFFF800

using std::placeholders::_1;
using std::placeholders::_2;

class SocketCAN_Sender : public rclcpp_lifecycle::LifecycleNode
{
    public:
        SocketCAN_Sender()
        : LifecycleNode("sender")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring...");

            this->declare_parameter<std::string>("interface_name", "can0");

            this->get_parameter("interface_name", interfaceName);

            if ((socketID = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to create socket: %d", socketID);
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }

            service = this->create_service<can_interface::srv::CanFrame>("socketcan/sender/input/data", std::bind(&SocketCAN_Sender::serviceCallback, this, _1, _2));

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuration completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activating...");

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempting to connect on: %s", interfaceName.c_str());
            
            strcpy(ifr.ifr_name, interfaceName.c_str());
            
            if(ioctl(socketID, SIOCGIFINDEX, &ifr) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to find interface: %s", std::strerror(errno));
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            if(!(ifr.ifr_flags & IFF_UP))
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Selected interface '%s' is not UP", interfaceName.c_str());

            memset(&addr, 0, sizeof(addr));
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if(bind(socketID, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to bind to socket: %s", std::strerror(errno));
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activation completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating...");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cleaning Up...");

            if(socketID != SOCKET_CLOSED_PROGRAMATICALLY && close(socketID) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to close socket: %s", std::strerror(errno));
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            socketID = SOCKET_CLOSED_PROGRAMATICALLY;

            service.reset();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cleanup completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting Down...");

            if(socketID != SOCKET_CLOSED_PROGRAMATICALLY && close(socketID) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to close socket: %s", std::strerror(errno));
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            socketID = SOCKET_CLOSED_PROGRAMATICALLY;

            service.reset();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shut down completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        ~SocketCAN_Sender()
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Destructing...");

            if(socketID != SOCKET_CLOSED_PROGRAMATICALLY && close(socketID) < 0)
            {
                RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to close socket: %s", std::strerror(errno));
            }
            socketID = SOCKET_CLOSED_PROGRAMATICALLY;

            service.reset();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Destructor completed successfully");
        }

        void serviceCallback (const std::shared_ptr<can_interface::srv::CanFrame::Request> request,
                            std::shared_ptr<can_interface::srv::CanFrame::Response> response)
        {
            int err;
            can_frame frame;

            if(request->frame.dlc > MAX_DLC_LENGTH)
            {
                response->status = response->STATUS_ERROR_INVALID_ID;
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                                "Message was sent with an invalid ID. IDE: %d - ID: %X", 
                                request->frame.is_extended_id, request->frame.can_id);
                err = -1;
            }

            if((request->frame.is_extended_id && (request->frame.can_id & CAN_ID_LARGER_THAN_29_BIT_MASK))
                || (!request->frame.is_extended_id && (request->frame.can_id & CAN_ID_LARGER_THAN_11_BIT_MASK)))
                {
                    response->status = response->STATUS_ERROR_DLC_OUT_OF_RANGE;
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                                "Message was sent with an invalid DLC: %d", 
                                request->frame.dlc);
                    err = -1;
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
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to write data: %s", std::strerror(errno));
                }
                else
                {
                    response->status = response->STATUS_ERROR_PARTIAL_FRAME_SENT;
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Could only write: %s bytes", std::strerror(errno));
                }

                return;
            }
            response->status = response->STATUS_OK;
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Frame sent successfully");
        }
    
    private:
        std::string interfaceName;
        int socketID;
        struct ifreq ifr;
        struct sockaddr_can addr;
        rclcpp::Service<can_interface::srv::CanFrame>::SharedPtr service;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<SocketCAN_Sender> lc_node =
        std::make_shared<SocketCAN_Sender>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}