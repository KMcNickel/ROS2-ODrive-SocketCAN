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
#include "can_interface/msg/can_frame.hpp"

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
            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Node created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuring...");

            this->declare_parameter<std::string>("interface_name", "can0");

            this->get_parameter("interface_name", interfaceName);

            if ((socketID = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("on_configure"), "Socket Error: Unable to create socket: %d", socketID);
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }

            subscription = this->create_subscription<can_interface::msg::CanFrame>("socketcan/sender/input/data", 10,
                std::bind(&SocketCAN_Sender::subscriptionCallback, this, _1));

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuration completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activating...");

            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Attempting to connect on: %s", interfaceName.c_str());
            
            strcpy(ifr.ifr_name, interfaceName.c_str());
            
            if(ioctl(socketID, SIOCGIFINDEX, &ifr) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("on_activate"), "Socket Error: Unable to find interface: %s", std::strerror(errno));
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            if(!(ifr.ifr_flags & IFF_UP))
                RCLCPP_WARN(rclcpp::get_logger("on_activate"), "Selected interface '%s' is not UP", interfaceName.c_str());

            memset(&addr, 0, sizeof(addr));
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if(bind(socketID, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("on_activate"), "Socket Error: Unable to bind to socket: %s", std::strerror(errno));
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activation completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivating...");
            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleaning Up...");

            if(socketID != SOCKET_CLOSED_PROGRAMATICALLY && close(socketID) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("on_cleanup"), "Socket Error: Unable to close socket: %s", std::strerror(errno));
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            socketID = SOCKET_CLOSED_PROGRAMATICALLY;

            subscription.reset();

            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleanup completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shutting Down...");

            if(socketID != SOCKET_CLOSED_PROGRAMATICALLY && close(socketID) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("on_shutdown"), "Socket Error: Unable to close socket: %s", std::strerror(errno));
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            socketID = SOCKET_CLOSED_PROGRAMATICALLY;

            subscription.reset();

            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shut down completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        ~SocketCAN_Sender()
        {
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutting down...");

            if(socketID != SOCKET_CLOSED_PROGRAMATICALLY && close(socketID) < 0)
            {
                RCLCPP_FATAL(rclcpp::get_logger("Destructor"), "Socket Error: Unable to close socket: %s", std::strerror(errno));
            }
            socketID = SOCKET_CLOSED_PROGRAMATICALLY;

            subscription.reset();

            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shut down completed successfully");
        }

        void subscriptionCallback(const can_interface::msg::CanFrame & incomingData)
        {
            int err = 0;
            can_frame outgoingFrame;

            RCLCPP_DEBUG(rclcpp::get_logger("subscriptionCallback"), "Data incoming");

            if(this->get_current_state().id() != 3)     //3 is the "active" lifecycle state
            {
                RCLCPP_DEBUG(rclcpp::get_logger("subscriptionCallback"), "Ignoring data while in current state");
                return;
            }

            if(incomingData.dlc > MAX_DLC_LENGTH)
            {
                RCLCPP_WARN(rclcpp::get_logger("subscriptionCallback"), 
                            "Message was sent with an invalid ID. IDE: %d - ID: %X", 
                            incomingData.is_extended_id, incomingData.can_id);
                err = -1;
            }

            if((incomingData.is_extended_id && (incomingData.can_id & CAN_ID_LARGER_THAN_29_BIT_MASK))
                    || (!incomingData.is_extended_id && (incomingData.can_id & CAN_ID_LARGER_THAN_11_BIT_MASK)))
            {
                RCLCPP_WARN(rclcpp::get_logger("subscriptionCallback"), 
                            "Message was sent with an invalid ID: %d", 
                            incomingData.can_id);
                err += -2;
            }
            
            if(err != 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("subscriptionCallback"), "Message will not be sent");
                return;
            }

            outgoingFrame.can_id = incomingData.is_extended_id;
            outgoingFrame.can_id = (outgoingFrame.can_id << 1) | incomingData.is_remote_request;
            outgoingFrame.can_id = (outgoingFrame.can_id << 1) | incomingData.is_error;
            outgoingFrame.can_id = (outgoingFrame.can_id << 29) | incomingData.can_id;
            outgoingFrame.can_dlc = incomingData.dlc;
            std::copy(std::begin(incomingData.data), std::end(incomingData.data), std::begin(outgoingFrame.data));

            RCLCPP_DEBUG(rclcpp::get_logger("subscriptionCallback"), "Sending:\n\tID: 0x%X\n\tLength: %d\n\tData: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
                    outgoingFrame.can_id, outgoingFrame.can_dlc, outgoingFrame.data[0], outgoingFrame.data[1], outgoingFrame.data[2],
                    outgoingFrame.data[3], outgoingFrame.data[4], outgoingFrame.data[5], outgoingFrame.data[6], outgoingFrame.data[7]);
                
            if((err = write(socketID, &outgoingFrame, sizeof(struct can_frame))) != sizeof(struct can_frame))
            {
                if(err < 0)
                    RCLCPP_ERROR(rclcpp::get_logger("subscriptionCallback"), "Socket Error: Unable to write data: %s", std::strerror(errno));
                else
                    RCLCPP_ERROR(rclcpp::get_logger("subscriptionCallback"), "Socket Error: Could only write: %d bytes", err);
                return;
            }
            RCLCPP_DEBUG(rclcpp::get_logger("subscriptionCallback"), "Frame sent successfully");
        }
    
    private:
        std::string interfaceName;
        int socketID;
        struct ifreq ifr;
        struct sockaddr_can addr;
        rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr subscription;
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