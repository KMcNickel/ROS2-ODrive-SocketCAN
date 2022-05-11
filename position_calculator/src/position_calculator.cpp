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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "odrive_interface/msg/axis_status.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose.hpp"

#define TRANSFORM_PUBLISH_TIMER_INTERVAL 100
#define DIAMETER_TO_DISTANCE_TRAVELLED(d) ((d * 3.14159) * (TRANSFORM_PUBLISH_TIMER_INTERVAL / 1000))

using std::placeholders::_1;

class PositionCalculator : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit PositionCalculator()
        : LifecycleNode("PositionCalculator")
        {
            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Node created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuring...");

            this->declare_parameter<std::string>("device_name", "default");
            this->declare_parameter<double>("propulsion_wheel_diameter", 1);

            this->get_parameter("device_name", deviceName);
            this->get_parameter("propulsion_wheel_diameter", propulsionWheelDiameter);

            powerCasterAPropulsionStatusSubscriber = this->create_subscription<odrive_interface::msg::AxisStatus>(
              "position/input/A/propulsion", 10, std::bind(&PositionCalculator::processPropulsionStatus, this, _1, 0));
            powerCasterBPropulsionStatusSubscriber = this->create_subscription<odrive_interface::msg::AxisStatus>(
              "position/input/B/propulsion", 10, std::bind(&PositionCalculator::processPropulsionStatus, this, _1, 1));
            powerCasterASteerAngleSubscriber = this->create_subscription<std_msgs::msg::Float64>(
              "position/input/A/angle", 10, std::bind(&PositionCalculator::processSteerAngle, this, _1, 0));
            powerCasterBSteerAngleSubscriber = this->create_subscription<std_msgs::msg::Float64>(
              "position/input/B/angle", 10, std::bind(&PositionCalculator::processSteerAngle, this, _1, 1));

              transformPublishTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(TRANSFORM_PUBLISH_TIMER_INTERVAL),
                std::bind(&PositionCalculator::publishTransform, this));

                transformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuration completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activating...");



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

        ~PositionCalculator()
        {
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutting Down");

            cleanUpVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutdown complete. Goodbye!");
        }

    private:
        void cleanUpVariables()
        {
          powerCasterAPropulsionStatusSubscriber.reset();
          powerCasterBPropulsionStatusSubscriber.reset();
          powerCasterASteerAngleSubscriber.reset();
          powerCasterBSteerAngleSubscriber.reset();

          lastPropulsionVelocity[0] = lastPropulsionVelocity[1] = 0;
          lastSteerAngle[0] = lastSteerAngle[1] = 0;
        }

        void processPropulsionStatus(odrive_interface::msg::AxisStatus status, int32_t casterNum)
        {
          if(this->get_current_state().id() != 3)    //3 is the ID for Active state
            return;

            RCLCPP_DEBUG(rclcpp::get_logger("processPropulsionStatus"), "New power caster %d velocity: %f", casterNum, status.current_velocity);

            lastPropulsionVelocity[casterNum] = status.current_velocity;
        }

        void processSteerAngle(std_msgs::msg::Float64 angle, int32_t casterNum)
        {
          if(this->get_current_state().id() != 3)    //3 is the ID for Active state
            return;

            RCLCPP_DEBUG(rclcpp::get_logger("processSteerAngle"), "New power caster %d angle: %f", casterNum, angle.data);

            lastPropulsionVelocity[casterNum] = angle.data;
        }

        void publishTransform()
        {
          geometry_msgs::msg::TransformStamped message;
          double x, y, r;

          RCLCPP_DEBUG(rclcpp::get_logger("publishTransform"), "Processing transform");

          message.header.stamp = this->get_clock()->now();
          message.header.frame_id = "world";
          message.child_frame_id = deviceName;

          double averageAngle = (lastSteerAngle[0] + lastSteerAngle[1]) / 2;
          double averageDistance = (DIAMETER_TO_DISTANCE_TRAVELLED(lastPropulsionVelocity[0])
              + DIAMETER_TO_DISTANCE_TRAVELLED(lastPropulsionVelocity[1])) / 2;

          if(averageAngle > 0 && averageAngle < 90);
          else if(averageAngle > 90 && averageAngle < 180);
          else if(averageAngle > 180 && averageAngle < 270);
          else if(averageAngle > 270 && averageAngle < 360);
          else if(averageAngle == 0)
          {
            x = 0;
            y = averageDistance;
            r = 0;
          }
          else if(averageAngle == 90)
          {
            x = averageDistance;
            y = 0;
            r = 0;
          }
          else if(averageAngle == 180)
          {
            x = 0;
            y = averageDistance * -1;
            r = 0;
          }
          else if(averageAngle == 270)
          {
            x = averageDistance * -1;
            y = 0;
            r = 0;
          }

          message.transform.translation.x = x;
          message.transform.translation.y = y;
          message.transform.translation.z = 0;

          tf2::Quaternion quat;
          quat.setRPY(0, 0, r);   //Use Roll, Pitch, Yaw (we can only change Yaw though)

          message.transform.rotation.x = quat.x();
          message.transform.rotation.y = quat.y();
          message.transform.rotation.z = quat.z();
          message.transform.rotation.w = quat.w();

          transformBroadcaster->sendTransform(message);
        }

        rclcpp::Subscription<odrive_interface::msg::AxisStatus>::SharedPtr powerCasterAPropulsionStatusSubscriber;
        rclcpp::Subscription<odrive_interface::msg::AxisStatus>::SharedPtr powerCasterBPropulsionStatusSubscriber;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr powerCasterASteerAngleSubscriber;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr powerCasterBSteerAngleSubscriber;

        rclcpp::TimerBase::SharedPtr transformPublishTimer;

        std::unique_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;

        std::string deviceName;
        double propulsionWheelDiameter;

        double lastPropulsionVelocity[2];
        double lastSteerAngle[2];
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<PositionCalculator> lc_node =
        std::make_shared<PositionCalculator>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}