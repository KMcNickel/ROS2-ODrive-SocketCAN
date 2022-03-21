#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <cerrno>
#include <cstring>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "can_interface/msg/can_frame.hpp"

#define SOCKET_CLOSED_PROGRAMATICALLY -10
#define PUBLISHER_QUEUE_SIZE 50
#define SOCKET_POLL_TIMEOUT 0       //in milliseconds. 0 returns immediately

class SocketCAN_Receiver : public rclcpp_lifecycle::LifecycleNode
{
    public:
        SocketCAN_Receiver()
        : LifecycleNode("receiver")
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

            pollDesc.fd = socketID;
            pollDesc.events = POLLIN;

            publisher = this->create_publisher<can_interface::msg::CanFrame>("socketcan/receiver/data", PUBLISHER_QUEUE_SIZE);

            timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&SocketCAN_Receiver::receiveData, this));

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

            publisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activation completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating...");
            publisher->on_deactivate();
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

            publisher.reset();

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

            publisher.reset();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shut down completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        ~SocketCAN_Receiver()
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Destructing...");

            if(socketID != SOCKET_CLOSED_PROGRAMATICALLY && close(socketID) < 0)
            {
                RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to close socket: %s", std::strerror(errno));
            }
            socketID = SOCKET_CLOSED_PROGRAMATICALLY;

            publisher.reset();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Destructor completed successfully");
        }

    private:
        void receiveData()
        {
            struct can_frame frame;
            auto message = can_interface::msg::CanFrame();
            int event;

            event = poll(&pollDesc, 1, SOCKET_POLL_TIMEOUT);

            if(event < 0 && rclcpp::ok() && errno != EINTR) //EINTR = Function interrupted (like if we Ctrl + C)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to poll socket: %s", std::strerror(errno));
                return;
            }
            if(event > 0)
            {
                if(read(socketID, &frame, sizeof(struct can_frame)) < 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket Error: Unable to read data: %s", std::strerror(errno));
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
    
        int socketID;
        std::string interfaceName;
        struct ifreq ifr;
        struct sockaddr_can addr;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp_lifecycle::LifecyclePublisher<can_interface::msg::CanFrame>::SharedPtr publisher;
        struct pollfd pollDesc;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<SocketCAN_Receiver> lc_node =
        std::make_shared<SocketCAN_Receiver>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}