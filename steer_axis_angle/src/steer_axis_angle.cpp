#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <thread>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "odrive_interface/msg/axis_status.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class SteerAxisAngle : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit SteerAxisAngle()
        : LifecycleNode("SteerAxisAngle")
        {
            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Node created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuring...");

            this->declare_parameter<int32_t>("axis_number", 0);
            this->declare_parameter<double>("axis_gear_ratio", 1.0);

            this->get_parameter<int32_t>("axis_number", axisNumber);
            this->get_parameter<double>("axis_gear_ratio", axisGearRatio);

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Using Axis %d with a gear ratio of %f:1", axisNumber, axisGearRatio);

            odriveAxisStatusSubscriber = this->create_subscription<odrive_interface::msg::AxisStatus>(
              "angle/input/odrive/status", 10, std::bind(&SteerAxisAngle::processOdriveData, this, _1));

            axisSteerAnglePublisher = this->create_publisher<std_msgs::msg::Float64>("angle/output/current", 10);

            zeroAxisService = this->create_service<std_srvs::srv::Trigger>(
              "angle/input/zero", std::bind(&SteerAxisAngle::zeroAxis, this, _1, _2));

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuration completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activating...");

            axisSteerAnglePublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activation completed successfully");
            
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivating...");

            axisSteerAnglePublisher->on_deactivate();

            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleaning Up...");

            cleanUpVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleanup completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shutting Down...");

            cleanUpVariables();

            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shut down completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        ~SteerAxisAngle()
        {
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutting Down");

            cleanUpVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutdown complete. Goodbye!");
        }

    private:
        void cleanUpVariables()
        {
          currentOffset = 0;

          odriveAxisStatusSubscriber.reset();
          axisSteerAnglePublisher.reset();
        }

        void processOdriveData(const odrive_interface::msg::AxisStatus status)
        {
          double angle;
          auto message = std_msgs::msg::Float64();

          if(this->get_current_state().id() != 3)     //3 is the state ID for Activated
            return;

          lastPosition = status.current_position;

          angle = ((lastPosition - currentOffset) / axisGearRatio) * 360.0;

          RCLCPP_DEBUG(rclcpp::get_logger("processODriveData"), "Current axis %d steer angle: %f degrees", axisNumber, angle);

          message.data = angle;

          axisSteerAnglePublisher->publish(message);
        }

        void zeroAxis(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          if(this->get_current_state().id() != 3)     //3 is the state ID for Activated
            return;

          RCLCPP_INFO(rclcpp::get_logger("processODriveData"), "Zeroing axis. Do not move the wheel...");
          RCLCPP_INFO(rclcpp::get_logger("processODriveData"), "Previous offset: %f", currentOffset);

          currentOffset = std::fmod(lastPosition, axisGearRatio);
          
          RCLCPP_INFO(rclcpp::get_logger("processODriveData"), "New offset: %f", currentOffset);
          RCLCPP_INFO(rclcpp::get_logger("processODriveData"), "Complete!");

          response->success = true;
        }

        rclcpp::Subscription<odrive_interface::msg::AxisStatus>::SharedPtr odriveAxisStatusSubscriber;

        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr axisSteerAnglePublisher;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroAxisService;

        int32_t axisNumber;
        mutable double axisGearRatio;

        double currentOffset;
        double lastPosition;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<SteerAxisAngle> lc_node =
        std::make_shared<SteerAxisAngle>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}