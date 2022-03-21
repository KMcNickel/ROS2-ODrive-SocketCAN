#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "can_interface/msg/can_frame.hpp"
#include "can_interface/srv/can_frame.hpp"
#include "odrive_interface/msg/heartbeat.hpp"
#include "odrive_interface/msg/encoder_estimates.hpp"
#include "odrive_interface/srv/input_position.hpp"
#include "odrive_interface/srv/axis_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ODrive : public rclcpp_lifecycle::LifecycleNode
{
    public:
        ODrive()
        : LifecycleNode("ODrive")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring...");

            this->declare_parameter<std::int32_t>("axis_number", 0);

            this->get_parameter("axis_number", axisNumber);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using Axis %d", axisNumber);

            createInterfaces();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuration completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activating...");

            heartbeatPublisher->on_activate();
            encoderEstimatePublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activation completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating...");

            heartbeatPublisher->on_deactivate();
            encoderEstimatePublisher->on_deactivate();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cleaning Up...");

            currentAxisState = ODRIVE_AXIS_STATE_Undefined;
            currentPosition = 0;
            currentVelocity = 0;

            canDataSenderClient.reset();
            heartbeatPublisher.reset();
            encoderEstimatePublisher.reset();
            canDataSubscription.reset();
            inputPositionService.reset();
            axisStateService.reset();
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cleanup completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting Down...");

            currentAxisState = ODRIVE_AXIS_STATE_Undefined;
            currentPosition = 0;
            currentVelocity = 0;

            canDataSenderClient.reset();
            heartbeatPublisher.reset();
            encoderEstimatePublisher.reset();
            canDataSubscription.reset();
            inputPositionService.reset();
            axisStateService.reset();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shut down completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        ~ODrive()
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting Down");

            currentAxisState = ODRIVE_AXIS_STATE_Undefined;
            currentPosition = 0;
            currentVelocity = 0;

            canDataSenderClient.reset();
            heartbeatPublisher.reset();
            encoderEstimatePublisher.reset();
            canDataSubscription.reset();
            inputPositionService.reset();
            axisStateService.reset();
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown complete. Goodbye!");
        }

        enum odrive_commands
        {
            ODRIVE_COMMAND_Heartbeat = 0x01,
            ODRIVE_COMMAND_Estop = 0x02,
            ODRIVE_COMMAND_GetMotorError = 0x03,
            ODRIVE_COMMAND_GetEncoderError = 0x04,
            ODRIVE_COMMAND_SensorlessError = 0x05,
            ODRIVE_COMMAND_SetAxisNodeID = 0x06,
            ODRIVE_COMMAND_SetAxisRequestedState = 0x07,
            ODRIVE_COMMAND_SetAxisStartupConfig = 0x08,
            ODRIVE_COMMAND_GetEncoderEstimates = 0x09,
            ODRIVE_COMMAND_GetEncoderCount = 0x0A,
            ODRIVE_COMMAND_SetControllerModes = 0x0B,
            ODRIVE_COMMAND_SetInputPosition = 0x0C,
            ODRIVE_COMMAND_SetInputVelocity = 0x0D,
            ODRIVE_COMMAND_SetInputTorque = 0x0E,
            ODRIVE_COMMAND_SetLimits = 0x0F,
            ODRIVE_COMMAND_StartAnticogging = 0x10,
            ODRIVE_COMMAND_SetTrajectoryVelocityLimit = 0x11,
            ODRIVE_COMMAND_SetTrajectoryAccelerationLimits = 0x12,
            ODRIVE_COMMAND_SetTrajectoryInertia = 0x13,
            ODRIVE_COMMAND_GetIQ = 0x14,
            ODRIVE_COMMAND_GetSensorlessEstimates = 0x15,
            ODRIVE_COMMAND_RebootODrive = 0x16,
            ODRIVE_COMMAND_GetVBusVoltage = 0x17,
            ODRIVE_COMMAND_ClearErrors = 0x18,
            ODRIVE_COMMAND_SetLinearCount = 0x19,
            ODRIVE_COMMAND_SetPositionGain = 0x1A,
            ODRIVE_COMMAND_SetVelocityGains = 0x1B,
        };

        enum odrive_axis_states
        {
            ODRIVE_AXIS_STATE_Undefined = 0x0,
            ODRIVE_AXIS_STATE_Idle = 0x01,
            ODRIVE_AXIS_STATE_StartupSequence = 0x02,
            ODRIVE_AXIS_STATE_FullCalibrationSequence = 0x03,
            ODRIVE_AXIS_STATE_MotorCalibration = 0x04,
            //There is no state 5. Because reasons...
            ODRIVE_AXIS_STATE_EncoderIndexSearch = 0x06,
            ODRIVE_AXIS_STATE_EncoderOffsetCalibration = 0x07,
            ODRIVE_AXIS_STATE_ClosedLoopControl = 0x08,
            ODRIVE_AXIS_STATE_LockinSpin = 0x09,
            ODRIVE_AXIS_STATE_EncoderDirFind = 0x0A,
            ODRIVE_AXIS_STATE_Homing = 0x0B,
            ODRIVE_AXIS_STATE_EncoderHallPolarityCalibration = 0x0C,
            ODRIVE_AXIS_STATE_EncoderHallPhaseCalibration = 0x0D
        };

        enum odrive_activate_sequence_types
        {
            ODRIVE_ACTIVATE_SEQUENCE_ImmediateCLosedLoop,   //Immediately go to closed loop control
            ODRIVE_ACTIVATE_SEQUENCE_EncoderIndexSearch,    //Run encoder index search, then go to closed loop
            ODRIVE_ACTIVATE_SEQUENCE_FullCalibration        //Run full calibration, then go to closed loop
        };

    private:
        void createInterfaces()
        {
            canDataSenderClient = this->create_client<can_interface::srv::CanFrame>("odrivecan/sender");
            heartbeatPublisher = this->create_publisher<odrive_interface::msg::Heartbeat>("odrive/status/heartbeat", 10);
            encoderEstimatePublisher = this->create_publisher<odrive_interface::msg::EncoderEstimates>("odrive/status/encoderEstimates", 10);
            canDataSubscription = this->create_subscription<can_interface::msg::CanFrame>(
                "odrivecan/receiver/data", 50, std::bind(&ODrive::canDataReceived, this, _1));
            inputPositionService = this->create_service<odrive_interface::srv::InputPosition>(
                "odrive/input/position", std::bind(&ODrive::setInputPosition, this, _1, _2));
            axisStateService = this->create_service<odrive_interface::srv::AxisState>(
                "odrive/input/axis_state", std::bind(&ODrive::setAxisState, this, _1, _2));
        }

        void setInputPosition(const std::shared_ptr<odrive_interface::srv::InputPosition::Request> request,
                            std::shared_ptr<odrive_interface::srv::InputPosition::Response> response)
        {
            auto canRequest = std::make_shared<can_interface::srv::CanFrame::Request>();

            canRequest->frame.can_id = (axisNumber << 5) | ODRIVE_COMMAND_SetInputPosition;
            canRequest->frame.is_error = canRequest->frame.is_extended_id = canRequest->frame.is_remote_request = 0;
            canRequest->frame.dlc = 8;
            memcpy(canRequest->frame.data.data(), &request->input_position, sizeof(request->input_position));

            auto result = canDataSenderClient->async_send_request(canRequest);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
            { 
                response->status = response->STATUS_OK;
            } else {
                response->status = response->STATUS_ERROR_OTHER_SERVICE_FAILED;
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Set input position failed because of an error from CAN service");
            }
        }

        void setAxisState(const std::shared_ptr<odrive_interface::srv::AxisState::Request> request,
                        std::shared_ptr<odrive_interface::srv::AxisState::Response> response)
        {
            auto canRequest = std::make_shared<can_interface::srv::CanFrame::Request>();

            canRequest->frame.can_id = (axisNumber << 5) | ODRIVE_COMMAND_SetAxisRequestedState;
            canRequest->frame.is_error = canRequest->frame.is_extended_id = canRequest->frame.is_remote_request = 0;
            canRequest->frame.dlc = 8;
            memcpy(canRequest->frame.data.data(), &request->state, sizeof(request->state));

            auto result = canDataSenderClient->async_send_request(canRequest);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
            { 
                response->status = response->STATUS_OK;
            } else {
                response->status = response->STATUS_ERROR_OTHER_SERVICE_FAILED;
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Set axis state failed because of an error from CAN service");
            }
        }

        void canDataReceived(const can_interface::msg::CanFrame & message) const
        {
            int32_t cmd_id;

            if(message.can_id >> 5 != (uint32_t) axisNumber)
                return;

            cmd_id = message.can_id & 0b00011111;

            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Processing message of command: %d", cmd_id);

            switch((odrive_commands) cmd_id)
            {
                case ODRIVE_COMMAND_Heartbeat:
                    UpdateHeartbeat(message.data);
                    break;
                case ODRIVE_COMMAND_GetEncoderEstimates:
                    UpdateEncoderEstimates(message.data);
                    break;
                default:
                    break;
            }
        }

        void UpdateHeartbeat(std::array<uint8_t, 8UL> canData) const
        {
            auto message = odrive_interface::msg::Heartbeat();

            message.axis_error = (canData[0] << 24) | (canData[1] << 16) | (canData[2] << 8) | canData[3];
            message.axis_state = canData[4];
            message.controller_status = canData[7];

            if(message.axis_state != currentAxisState)
            {
                currentAxisState = (odrive_axis_states) message.axis_state;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis state changed to: %d", currentAxisState);
            }

            heartbeatPublisher->publish(message);
        }

        void UpdateEncoderEstimates(std::array<uint8_t, 8UL> canData) const
        {
            auto message = odrive_interface::msg::EncoderEstimates();

            memcpy(&currentPosition, canData.data(), sizeof(message.position_estimate));
            memcpy(&currentVelocity, canData.data() + 4, sizeof(message.velocity_estimate));

            message.position_estimate = currentPosition;
            message.velocity_estimate = currentVelocity;

            encoderEstimatePublisher->publish(message);
        }

        rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr canDataSubscription;

        rclcpp::Client<can_interface::srv::CanFrame>::SharedPtr canDataSenderClient;

        rclcpp::Service<odrive_interface::srv::InputPosition>::SharedPtr inputPositionService;
        rclcpp::Service<odrive_interface::srv::AxisState>::SharedPtr axisStateService;

        rclcpp_lifecycle::LifecyclePublisher<odrive_interface::msg::Heartbeat>::SharedPtr heartbeatPublisher;
        rclcpp_lifecycle::LifecyclePublisher<odrive_interface::msg::EncoderEstimates>::SharedPtr encoderEstimatePublisher;

        int32_t axisNumber;
        mutable odrive_axis_states currentAxisState;
        mutable float currentPosition;
        mutable float currentVelocity;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<ODrive> lc_node =
        std::make_shared<ODrive>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}