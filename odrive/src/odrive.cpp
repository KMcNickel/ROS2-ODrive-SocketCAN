#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <thread>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "can_interface/msg/can_frame.hpp"
#include "can_interface/srv/can_frame.hpp"
#include "std_msgs/msg/float64.hpp"
#include "odrive_interface/msg/axis_status.hpp"
#include "odrive_interface/srv/input_position.hpp"
#include "odrive_interface/srv/input_velocity.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

#define STATUS_PUBLISH_TIMER_INTERVAL 50
#define CAN_SERVICE_WAIT_TIMEOUT 50
#define ODRIVE_TO_US_WATCHDOG_INTERVAL 100
#define MAINTENANCE_TIMER_INTERVAL 10
#define CAN_DLC_MAX_SIZE 8
#define MILLISECOND_TO_SECOND(a) ((double) a / 1000)

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
            this->declare_parameter<bool>("publish_debug_messages", false);

            this->get_parameter("axis_number", axisNumber);
            this->get_parameter("calibration_type", calibrationType);
            this->get_parameter("publish_debug_messages", publishDebugMessages);

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
            watchdogTimerMonitorPublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activation completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating...");

            axisStatusPublisher->on_deactivate();
            watchdogTimerMonitorPublisher->on_deactivate();

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

        struct odrive_node_errors
        {
            int8_t BoardError;
            int32_t AxisError;
            int64_t MotorError;
            int8_t ControllerError;
            int16_t EncoderError;
            int8_t SensorlessError;

            bool WatchdogFromODriveTimeoutError;
            bool CANSenderServiceError;

            odrive_node_errors()
            {
                BoardError = AxisError = MotorError = ControllerError = 
                EncoderError = SensorlessError = 0;

                WatchdogFromODriveTimeoutError = 
                CANSenderServiceError = false;
            }

            bool ODriveErrorsExist() 
            {
                return BoardError && AxisError && MotorError && 
                    ControllerError && EncoderError && SensorlessError;
            }

            bool NodeErrorsExist()
            {
                return WatchdogFromODriveTimeoutError && CANSenderServiceError;
            }

            bool ErrorsExist()
            {
                return ODriveErrorsExist() && NodeErrorsExist();
            }
        };

    private:
        void createInterfaces()
        {
            canClientCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);

            canDataSenderClient = this->create_client<can_interface::srv::CanFrame>("odrive/output/can",
                            rmw_qos_profile_services_default, canClientCallbackGroup);
            axisStatusPublisher = this->create_publisher<odrive_interface::msg::AxisStatus>("odrive/output/status", 10);
            watchdogTimerMonitorPublisher = this->create_publisher<std_msgs::msg::Float64>("odrive/debug/output/watchdog", 10);
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
            rebootBoardService = this->create_service<std_srvs::srv::Trigger>(
                "odrive/input/reboot", std::bind(&ODrive::rebootBoard, this, _1, _2));
        }

        void createTimers()
        {
            statusPublishTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(STATUS_PUBLISH_TIMER_INTERVAL),
                    std::bind(&ODrive::SendAxisStatusMessage, this));
            maintenanceTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(MAINTENANCE_TIMER_INTERVAL),
                    std::bind(&ODrive::ExecuteMaintenanceFunctions, this));

            lastIncomingMessageTimeFromOdrive = this->now();
        }

        void resetVariables()
        {
            currentAxisState = ODRIVE_AXIS_STATE_Undefined;
            currentPosition = 0;
            currentVelocity = 0;
            currentAxisStatus = ODRIVE_AXIS_STATUS_NOT_READY;
            currentSystemErrors = odrive_node_errors();

            canDataSubscription.reset();
            canDataSenderClient.reset();
            inputPositionService.reset();
            inputVelocityService.reset();
            axisStartupService.reset();
            axisShutdownService.reset();
            clearErrorService.reset();
            axisStatusPublisher.reset();
            statusPublishTimer.reset();
            maintenanceTimer.reset();
            canClientCallbackGroup.reset();

            lastIncomingMessageTimeFromOdrive = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
        }

        void ExecuteMaintenanceFunctions()
        {
            double watchdogDuration = (this->now() - lastIncomingMessageTimeFromOdrive).seconds();
            auto message = std_msgs::msg::Float64();
            message.data = watchdogDuration;

            if(watchdogTimerMonitorPublisher->is_activated())
                watchdogTimerMonitorPublisher->publish(message);

            if(currentAxisStatus != ODRIVE_AXIS_STATUS_ERROR && currentAxisStatus != ODRIVE_AXIS_STATUS_NOT_READY
                && (watchdogDuration > MILLISECOND_TO_SECOND(ODRIVE_TO_US_WATCHDOG_INTERVAL)))
                WatchdogExpired(watchdogDuration);

            SendMaintenanceMessages();
        }

        void WatchdogExpired(float duration)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ODrive Board Watchdog timed out after %f seconds", duration);
            currentAxisStatus = ODRIVE_AXIS_STATUS_ERROR;

            currentSystemErrors.WatchdogFromODriveTimeoutError = true;

            sendAxisStateRequest(ODRIVE_AXIS_STATE_Idle);

            SendCanData(ODRIVE_COMMAND_Estop, 0, 0);
        }

        void SendMaintenanceMessages()
        {

        }

        void startupAxis(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            bool axisChangeSucceeded;
            std::string axisChangeMessage;

            switch(currentAxisStatus)
            {
                case ODRIVE_AXIS_STATUS_NOT_READY:
                    switch(calibrationType)
                    {
                        case ODRIVE_AXIS_CALIBRATION_None:
                            std::tie(axisChangeSucceeded, axisChangeMessage) = sendAxisStateRequest(ODRIVE_AXIS_STATE_ClosedLoopControl);
                            break;
                        case ODRIVE_AXIS_CALIBRATION_EncoderIndex:
                            std::tie(axisChangeSucceeded, axisChangeMessage) = sendAxisStateRequest(ODRIVE_AXIS_STATE_EncoderIndexSearch);
                            break;
                        case ODRIVE_AXIS_CALIBRATION_Full:
                            std::tie(axisChangeSucceeded, axisChangeMessage) = sendAxisStateRequest(ODRIVE_AXIS_STATE_FullCalibrationSequence);
                            break;
                    }
                    if(axisChangeSucceeded)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting axis calibration");
                        currentAxisStatus = ODRIVE_AXIS_STATUS_CALIBRATING;
                    }
                    response->success = axisChangeSucceeded;
                    response->message = axisChangeMessage;
                    break;
                case ODRIVE_AXIS_STATUS_IDLE:
                    
                    std::tie(axisChangeSucceeded, axisChangeMessage) = sendAxisStateRequest(ODRIVE_AXIS_STATE_ClosedLoopControl);
                    if(axisChangeSucceeded)
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting axis in closed loop");
                    response->success = axisChangeSucceeded;
                    response->message = axisChangeMessage;
                    break;
                default:
                    response->success = false;
                    response->message = "System status is not correct to start axis";
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Not starting axis due to incorrect status: %d", currentAxisStatus);
                    break;
            }
        }

        void shutdownAxis(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            std::tie(response->success, response->message) = sendAxisStateRequest(ODRIVE_AXIS_STATE_Idle);
        }

        void clearErrors(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            std::tie(response->success, response->message) = attemptToClearErrors();
        }

        std::tuple<bool, std::string> attemptToClearErrors()
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Clearing errors");

            if((this->now() - lastIncomingMessageTimeFromOdrive).seconds() <
                    MILLISECOND_TO_SECOND(ODRIVE_TO_US_WATCHDOG_INTERVAL))
                currentSystemErrors.WatchdogFromODriveTimeoutError = false;

            if(canDataSenderClient->service_is_ready())
                currentSystemErrors.CANSenderServiceError = false;

            if(!currentSystemErrors.NodeErrorsExist())
                return SendCanData(ODRIVE_COMMAND_ClearErrors, 0, 0);
            else
                return std::make_tuple(false, "Node errors exist that cannot be cleared");
        }

        void rebootBoard(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                std::tie(response->success, response->message) = sendRebootCommand();
            }

        std::tuple<bool, std::string> sendRebootCommand()
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending reboot command");

            if(currentAxisStatus != ODRIVE_AXIS_STATUS_ERROR)
                currentAxisStatus = ODRIVE_AXIS_STATUS_NOT_READY;

            return SendCanData(ODRIVE_COMMAND_RebootODrive, 0, 0);
        }

        void setInputPosition(const std::shared_ptr<odrive_interface::srv::InputPosition::Request> request,
                            std::shared_ptr<odrive_interface::srv::InputPosition::Response> response)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending new input velocity request:\n\tPosition: %f\n\tVelocity_FF: %d\n\tTorque_FF: %d",
                request->input_position, request->velocity_feedforward, request->torque_feedforward);

            std::tie(response->success, response->message) = sendSetInputPosition(
                request->input_position, request->velocity_feedforward, request->torque_feedforward);
        }

        std::tuple<bool, std::string> sendSetInputPosition(float position, int16_t velocity_ff, int16_t torque_ff)
        {
            if(currentSystemErrors.ErrorsExist())
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send new input position. Errors exist");
                return std::make_tuple(false, "Errors exist");
            }
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Received new input position request:\n\tPosition: %f\n\tVelocity_FF: %d\n\tTorque_FF: %d",
                position, velocity_ff, torque_ff);

            int8_t data[8];

            memcpy(data, &position, sizeof(position));
            memcpy(data + 4, &velocity_ff, sizeof(velocity_ff));
            memcpy(data + 6, &torque_ff, sizeof(torque_ff));

            return SendCanData(ODRIVE_COMMAND_SetInputPosition, data, sizeof(data));
        }

        void setInputVelocity(const std::shared_ptr<odrive_interface::srv::InputVelocity::Request> request,
                            std::shared_ptr<odrive_interface::srv::InputVelocity::Response> response)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Received new input velocity request:\n\tVelocity: %f\n\tTorque_FF: %d",
                request->input_velocity, request->torque_feedforward);

            std::tie(response->success, response->message) = sendSetInputVelocity(request->input_velocity, request->torque_feedforward);
        }

        std::tuple<bool, std::string> sendSetInputVelocity(float velocity, int16_t torque_ff)
        {
            if(currentSystemErrors.ErrorsExist())
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send new input velocity. Errors exist");
                return std::make_tuple(false, "Errors exist");
            }
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending new input velocity request:\n\tVelocity: %f\n\tTorque_FF: %d",
                velocity, torque_ff);

            int8_t data[8];

            memcpy(data, &velocity, sizeof(velocity));
            memcpy(data + 4, &torque_ff, sizeof(torque_ff));

            return SendCanData(ODRIVE_COMMAND_SetInputVelocity, data, sizeof(data));
        }

        std::tuple<bool, std::string> sendAxisStateRequest(odrive_axis_states new_state)
        {
            bool velSuccess;
            std::string velMessage;

            if(currentSystemErrors.ErrorsExist())
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send new axis state. Errors exist");
                return std::make_tuple(false, "Errors exist");
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting velocity to 0");

            std::tie(velSuccess, velMessage) = sendSetInputVelocity(0, 0);

            if(!velSuccess)
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send velocity.");
                return std::make_tuple(velSuccess, velMessage);
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending new state request: %d", (int32_t) new_state);

            int8_t data[8];

            memcpy(data, &new_state, sizeof(int32_t));

            return SendCanData(ODRIVE_COMMAND_SetAxisRequestedState, data, sizeof(int32_t));
        }

        std::tuple<bool, std::string> SendCanData(odrive_commands command, int8_t * data, int8_t dataLen)
        {
            auto canRequest = std::make_shared<can_interface::srv::CanFrame::Request>();

            if(dataLen > CAN_DLC_MAX_SIZE)
                return std::make_tuple(false, "Data field is too long");

            canRequest->frame.can_id = (axisNumber << 5) | command;
            canRequest->frame.is_error = canRequest->frame.is_extended_id = canRequest->frame.is_remote_request = 0;
            canRequest->frame.dlc = CAN_DLC_MAX_SIZE;
            if(dataLen > 0)
                memcpy(canRequest->frame.data.data(), data, dataLen);

            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending data to CAN");

            if(!canDataSenderClient->service_is_ready())
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN service is not ready");
                return std::make_tuple(false, "CAN service is not ready");
            }

            auto future = canDataSenderClient->async_send_request(canRequest);

            if(future.wait_for(std::chrono::milliseconds(CAN_SERVICE_WAIT_TIMEOUT)) == std::future_status::ready)
            {
                if(future.get().get()->status == future.get().get()->STATUS_OK)
                    return std::make_tuple(true, "");
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN service return status code: %d", future.get().get()->status);
                    return std::make_tuple(false, "CAN service returned an error");
                }
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN service did not respond");
                return std::make_tuple(false, "CAN service timed out");
            }
        }

        void canDataReceived(const can_interface::msg::CanFrame & message)
        {
            int32_t cmd_id;

            if(message.can_id >> 5 != (uint32_t) axisNumber)
                return;

            lastIncomingMessageTimeFromOdrive = this->now();

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
            int32_t oldAxisError = currentSystemErrors.AxisError;
            currentSystemErrors.AxisError = (canData[3] << 24) | (canData[2] << 16) | (canData[1] << 8) | canData[0];
            odrive_axis_states oldAxisState = currentAxisState;
            currentAxisState = (odrive_axis_states) canData[4];
            //canData[7] is the controller status flags. Still trying to find out what they mean

            if(oldAxisError != currentSystemErrors.AxisError)
            {
                if(currentSystemErrors.ErrorsExist())
                    currentAxisStatus = ODRIVE_AXIS_STATUS_ERROR;
                else
                    currentAxisStatus = ODRIVE_AXIS_STATUS_NOT_READY;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis error changed to: %d", currentSystemErrors.AxisError);
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
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rebootBoardService;

        rclcpp_lifecycle::LifecyclePublisher<odrive_interface::msg::AxisStatus>::SharedPtr axisStatusPublisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr watchdogTimerMonitorPublisher;

        rclcpp::TimerBase::SharedPtr statusPublishTimer;
        rclcpp::TimerBase::SharedPtr maintenanceTimer;

        rclcpp::CallbackGroup::SharedPtr canClientCallbackGroup;

        rclcpp::Time lastIncomingMessageTimeFromOdrive;

        int32_t axisNumber;
        int32_t calibrationType;
        bool publishDebugMessages;
        mutable odrive_axis_states currentAxisState;
        mutable odrive_axis_status currentAxisStatus;
        mutable float currentPosition;
        mutable float currentVelocity;
        odrive_node_errors currentSystemErrors;
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