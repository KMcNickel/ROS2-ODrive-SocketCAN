#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "can_interface/msg/can_frame.hpp"
#include "can_interface/srv/can_frame.hpp"
#include "odrive_interface/msg/axis_status.hpp"
#include "odrive_interface/srv/input_position.hpp"
#include "odrive_interface/srv/input_velocity.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

#define STATUS_PUBLISH_TIMER_INTERVAL 50

class ODrive : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit ODrive()
        : LifecycleNode("ODrive")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring...");

            this->declare_parameter<std::int32_t>("axis_number", 0);
            this->declare_parameter<std::int32_t>("calibration_type", 0);

            this->get_parameter("axis_number", axisNumber);
            this->get_parameter("calibration_type", calibrationType);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using Axis %d", axisNumber);

            createInterfaces();
            createTimers();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuration completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activating...");

            axisStatusPublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activation completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating...");

            axisStatusPublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cleaning Up...");

            resetVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cleanup completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting Down...");

            resetVariables();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shut down completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        ~ODrive()
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting Down");

            resetVariables();
            
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

        enum odrive_calibration_types
        {
            ODRIVE_AXIS_CALIBRATION_None,
            ODRIVE_AXIS_CALIBRATION_EncoderIndex,
            ODRIVE_AXIS_CALIBRATION_Full
        };

        enum odrive_axis_status
        {
            ODRIVE_AXIS_STATUS_ERROR = -1,
            ODRIVE_AXIS_STATUS_NOT_READY = 0,
            ODRIVE_AXIS_STATUS_CALIBRATING = 1,
            ODRIVE_AXIS_STATUS_IDLE = 2,
            ODRIVE_AXIS_STATUS_RUNNING = 3
        };

    private:
        void createInterfaces()
        {
            canDataSenderClient = this->create_client<can_interface::srv::CanFrame>("odrive/output/can");
            axisStatusPublisher = this->create_publisher<odrive_interface::msg::AxisStatus>("odrive/output/status", 10);
            canDataSubscription = this->create_subscription<can_interface::msg::CanFrame>(
                "odrive/input/can", 50, std::bind(&ODrive::canDataReceived, this, _1));
            inputPositionService = this->create_service<odrive_interface::srv::InputPosition>(
                "odrive/input/position", std::bind(&ODrive::setInputPosition, this, _1, _2));
            inputVelocityService = this->create_service<odrive_interface::srv::InputVelocity>(
                "odrive/input/velocity", std::bind(&ODrive::setInputVelocity, this, _1, _2));
            axisStartupService = this->create_service<std_srvs::srv::Trigger>(
                "odrive/input/start", std::bind(&ODrive::startupAxis, this, _1, _2));
            axisShutdownService = this->create_service<std_srvs::srv::Trigger>(
                "odrive/input/shutdown", std::bind(&ODrive::shutdownAxis, this, _1, _2));
            clearErrorService = this->create_service<std_srvs::srv::Trigger>(
                "odrive/input/clearErrors", std::bind(&ODrive::clearErrors, this, _1, _2));
        }

        void createTimers()
        {
            statusPublishTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(STATUS_PUBLISH_TIMER_INTERVAL),
                    std::bind(&ODrive::SendAxisStatusMessage, this));
        }

        void resetVariables()
        {
            currentAxisState = ODRIVE_AXIS_STATE_Undefined;
            currentPosition = 0;
            currentVelocity = 0;
            currentAxisError = 0;
            currentAxisStatus = ODRIVE_AXIS_STATUS_NOT_READY;

            canDataSenderClient.reset();
            axisStatusPublisher.reset();
            canDataSubscription.reset();
            inputPositionService.reset();

            statusPublishTimer.reset();
        }

        void startupAxis(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            if(currentAxisState == ODRIVE_AXIS_STATE_Idle)
            {
                switch(currentAxisStatus)
                {
                    case ODRIVE_AXIS_STATUS_NOT_READY:
                        switch(calibrationType)
                        {
                            case ODRIVE_AXIS_CALIBRATION_None:
                                sendAxisStateRequest(ODRIVE_AXIS_STATE_ClosedLoopControl);
                                break;
                            case ODRIVE_AXIS_CALIBRATION_EncoderIndex:
                                sendAxisStateRequest(ODRIVE_AXIS_STATE_EncoderIndexSearch);
                                break;
                            case ODRIVE_AXIS_CALIBRATION_Full:
                                sendAxisStateRequest(ODRIVE_AXIS_STATE_FullCalibrationSequence);
                                break;
                        }
                        currentAxisStatus = ODRIVE_AXIS_STATUS_CALIBRATING;
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting axis calibration");
                        response->success = true;
                        break;
                    case ODRIVE_AXIS_STATUS_IDLE:
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting axis in closed loop");
                        sendAxisStateRequest(ODRIVE_AXIS_STATE_ClosedLoopControl);
                        response->success = true;
                        break;
                    default:
                        response->success = false;
                        response->message = "System status is not correct to start axis";
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Not starting axis due to incorrect status: %d", currentAxisStatus);
                        break;
                }
            }
        }

        void shutdownAxis(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            sendAxisStateRequest(ODRIVE_AXIS_STATE_Idle);
            response->success = true;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down axis");
        }

        void clearErrors(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            sendClearErrorRequest();
            currentAxisStatus = ODRIVE_AXIS_STATUS_NOT_READY;       //Hacky since the Odrive board errors are not reportable
            response->success = true;
        }

        void setInputPosition(const std::shared_ptr<odrive_interface::srv::InputPosition::Request> request,
                            std::shared_ptr<odrive_interface::srv::InputPosition::Response> response)
        {
            auto canRequest = std::make_shared<can_interface::srv::CanFrame::Request>();

            canRequest->frame.can_id = (axisNumber << 5) | ODRIVE_COMMAND_SetInputPosition;
            canRequest->frame.is_error = canRequest->frame.is_extended_id = canRequest->frame.is_remote_request = 0;
            canRequest->frame.dlc = 8;
            memcpy(canRequest->frame.data.data(), &request->input_position, sizeof(request->input_position));
            memcpy(canRequest->frame.data.data() + 4, &request->velocity_feedforward, sizeof(request->velocity_feedforward));
            memcpy(canRequest->frame.data.data() + 6, &request->torque_feedforward, sizeof(request->torque_feedforward));

            response->status = response->STATUS_OK;

            canDataSenderClient->async_send_request(canRequest);
        }

        void setInputVelocity(const std::shared_ptr<odrive_interface::srv::InputVelocity::Request> request,
                            std::shared_ptr<odrive_interface::srv::InputVelocity::Response> response)
        {
            auto canRequest = std::make_shared<can_interface::srv::CanFrame::Request>();

            canRequest->frame.can_id = (axisNumber << 5) | ODRIVE_COMMAND_SetInputVelocity;
            canRequest->frame.is_error = canRequest->frame.is_extended_id = canRequest->frame.is_remote_request = 0;
            canRequest->frame.dlc = 8;
            memcpy(canRequest->frame.data.data(), &request->input_velocity, sizeof(request->input_velocity));
            memcpy(canRequest->frame.data.data() + 4, &request->torque_feedforward, sizeof(request->torque_feedforward));

            response->status = response->STATUS_OK;

            canDataSenderClient->async_send_request(canRequest);
        }

        void sendClearErrorRequest()
        {
            auto canRequest = std::make_shared<can_interface::srv::CanFrame::Request>();

            canRequest->frame.can_id = (axisNumber << 5) | ODRIVE_COMMAND_ClearErrors;
            canRequest->frame.is_error = canRequest->frame.is_extended_id = canRequest->frame.is_remote_request = 0;
            canRequest->frame.dlc = 8;

            canDataSenderClient->async_send_request(canRequest);
        }

        void sendAxisStateRequest(odrive_axis_states new_state)
        {
            auto canRequest = std::make_shared<can_interface::srv::CanFrame::Request>();

            canRequest->frame.can_id = (axisNumber << 5) | ODRIVE_COMMAND_SetAxisRequestedState;
            canRequest->frame.is_error = canRequest->frame.is_extended_id = canRequest->frame.is_remote_request = 0;
            canRequest->frame.dlc = 8;
            memcpy(canRequest->frame.data.data(), &new_state, sizeof(new_state));

            canDataSenderClient->async_send_request(canRequest);

            //return rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS;
        }

        void canDataReceived(const can_interface::msg::CanFrame & message)
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

        void UpdateHeartbeat(std::array<uint8_t, 8UL> canData)
        {
            int32_t oldAxisError = currentAxisError;
            currentAxisError = (canData[0] << 24) | (canData[1] << 16) | (canData[2] << 8) | canData[3];
            odrive_axis_states oldAxisState = currentAxisState;
            currentAxisState = (odrive_axis_states) canData[4];
            //canData[7] is the controller status flags. Still trying to find out what they mean

            if(oldAxisError != currentAxisError)
            {
                if(currentAxisError != 0)
                    currentAxisStatus = ODRIVE_AXIS_STATUS_ERROR;
                if(currentAxisError == 0)
                    currentAxisStatus = ODRIVE_AXIS_STATUS_NOT_READY;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis error changed to: %d", currentAxisError);
            }

            if(oldAxisState != currentAxisState)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis state changed to: %d", currentAxisState);

                if(currentAxisStatus == ODRIVE_AXIS_STATUS_CALIBRATING && currentAxisState == ODRIVE_AXIS_STATE_Idle)
                    sendAxisStateRequest(ODRIVE_AXIS_STATE_ClosedLoopControl);

                if(currentAxisState == ODRIVE_AXIS_STATE_ClosedLoopControl)
                    currentAxisStatus = ODRIVE_AXIS_STATUS_RUNNING;

                if(currentAxisStatus == ODRIVE_AXIS_STATUS_RUNNING && currentAxisState == ODRIVE_AXIS_STATE_Idle)
                    currentAxisStatus = ODRIVE_AXIS_STATUS_IDLE;
            }
        }

        void UpdateEncoderEstimates(std::array<uint8_t, 8UL> canData) const
        {
            memcpy(&currentPosition, canData.data(), sizeof(currentPosition));
            memcpy(&currentVelocity, canData.data() + 4, sizeof(currentVelocity));
        }

        void SendAxisStatusMessage()
        {
            auto message = odrive_interface::msg::AxisStatus();

            message.axis_status = currentAxisStatus;
            message.current_position = currentPosition;
            message.current_velocity = currentVelocity;

            if(axisStatusPublisher->is_activated())
                axisStatusPublisher->publish(message);
        }

        rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr canDataSubscription;

        rclcpp::Client<can_interface::srv::CanFrame>::SharedPtr canDataSenderClient;

        rclcpp::Service<odrive_interface::srv::InputPosition>::SharedPtr inputPositionService;
        rclcpp::Service<odrive_interface::srv::InputVelocity>::SharedPtr inputVelocityService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr axisStartupService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr axisShutdownService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clearErrorService;

        rclcpp_lifecycle::LifecyclePublisher<odrive_interface::msg::AxisStatus>::SharedPtr axisStatusPublisher;

        rclcpp::TimerBase::SharedPtr statusPublishTimer;

        int32_t axisNumber;
        int32_t calibrationType;
        mutable int32_t currentAxisError;
        mutable odrive_axis_states currentAxisState;
        mutable odrive_axis_status currentAxisStatus;
        mutable float currentPosition;
        mutable float currentVelocity;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<ODrive> lc_node =
        std::make_shared<ODrive>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}