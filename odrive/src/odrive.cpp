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
#include "std_msgs/msg/float64.hpp"
#include "odrive_interface/msg/axis_status.hpp"
#include "odrive_interface/srv/input_velocity.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

#define STATUS_PUBLISH_TIMER_INTERVAL 50
#define CAN_SERVICE_WAIT_TIMEOUT 100
#define ODRIVE_TO_US_WATCHDOG_INTERVAL 500
#define MAINTENANCE_TIMER_INTERVAL 10
#define ERROR_REQUEST_TIMER_INTERVAL 100
#define INCOMING_VELOCITY_WATCHDOG_TIMEOUT 250
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
            bool stateSuccess;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activating...");

            axisStatusPublisher->on_activate();
            canDataPublisher->on_activate();
            watchdogTimerMonitorPublisher->on_activate();

            stateSuccess = sendAxisStateRequest(ODRIVE_AXIS_STATE_Idle);

            if(stateSuccess)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activation completed successfully");
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activation failed to set axis state");
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating...");

            axisStatusPublisher->on_deactivate();
            canDataPublisher->on_deactivate();
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
            ODRIVE_COMMAND_GetSensorlessError = 0x05,
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

        enum odrive_system_status
        {
            ODRIVE_SYSTEM_STATUS_ERROR = -1,
            ODRIVE_SYSTEM_STATUS_NOT_READY = 0,
            ODRIVE_SYSTEM_STATUS_CALIBRATING = 1,
            ODRIVE_SYSTEM_STATUS_IDLE = 2,
            ODRIVE_SYSTEM_STATUS_RUNNING = 3
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
            bool IncomingVelocityTimeoutError;
            bool IncomingVelocityTimeoutErrorLatch;

            odrive_node_errors()
            {
                BoardError = AxisError = MotorError = ControllerError = 
                EncoderError = SensorlessError = 0;

                WatchdogFromODriveTimeoutError = 
                IncomingVelocityTimeoutError = 
                IncomingVelocityTimeoutErrorLatch = false;
            }

            bool ODriveErrorsExist() 
            {
                return BoardError || AxisError || MotorError || 
                    ControllerError || EncoderError || SensorlessError;
            }

            bool NodeErrorsExist()
            {
                return WatchdogFromODriveTimeoutError ||
                IncomingVelocityTimeoutError || IncomingVelocityTimeoutErrorLatch;
            }

            bool ErrorsExist()
            {
                return ODriveErrorsExist() || NodeErrorsExist();
            }
        };

    private:
        void createInterfaces()
        {
            canDataPublisher = this->create_publisher<can_interface::msg::CanFrame>("odrive/output/can", 10);
            axisStatusPublisher = this->create_publisher<odrive_interface::msg::AxisStatus>("odrive/output/status", 10);
            watchdogTimerMonitorPublisher = this->create_publisher<std_msgs::msg::Float64>("odrive/debug/output/watchdog", 10);
            canDataSubscription = this->create_subscription<can_interface::msg::CanFrame>(
                "odrive/input/can", 50, std::bind(&ODrive::canDataReceived, this, _1));
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
            errorRequestTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(ERROR_REQUEST_TIMER_INTERVAL),
                    std::bind(&ODrive::RequestErrors, this));
            incomingVelocityTimeoutTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(INCOMING_VELOCITY_WATCHDOG_TIMEOUT),
                    std::bind(&ODrive::incomingVelocityWatchdogElapsed, this));

            lastIncomingMessageTimeFromOdrive = this->now();
        }

        void resetVariables()
        {
            currentAxisState = ODRIVE_AXIS_STATE_Undefined;
            currentPosition = 0;
            currentVelocity = 0;
            ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_NOT_READY);
            currentSystemErrors = odrive_node_errors();

            canDataSubscription.reset();
            canDataPublisher.reset();
            inputVelocityService.reset();
            axisStartupService.reset();
            axisShutdownService.reset();
            clearErrorService.reset();
            axisStatusPublisher.reset();
            statusPublishTimer.reset();
            maintenanceTimer.reset();
            errorRequestTimer.reset();
            incomingVelocityTimeoutTimer.reset();

            lastIncomingMessageTimeFromOdrive = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
            lastIncomingVelocityMessage = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
        }

        void ExecuteMaintenanceFunctions()
        {
            double watchdogDuration = (this->now() - lastIncomingMessageTimeFromOdrive).seconds();
            auto message = std_msgs::msg::Float64();
            message.data = watchdogDuration;

            if(watchdogTimerMonitorPublisher->is_activated())
                watchdogTimerMonitorPublisher->publish(message);

            if(currentSystemStatus != ODRIVE_SYSTEM_STATUS_ERROR && currentSystemStatus != ODRIVE_SYSTEM_STATUS_NOT_READY
                && (watchdogDuration > MILLISECOND_TO_SECOND(ODRIVE_TO_US_WATCHDOG_INTERVAL)))
                WatchdogExpired(watchdogDuration);

            if(currentSystemStatus == ODRIVE_SYSTEM_STATUS_ERROR && !currentSystemErrors.ErrorsExist())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "All errors have been cleared");
                sendAxisStateRequest(ODRIVE_AXIS_STATE_Idle);
                ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_NOT_READY);
            }
        }

        void ChangeSystemStatus(odrive_system_status newStatus)
        {
            if(newStatus == currentSystemStatus) return;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Changing status from %d to %d", currentSystemStatus, newStatus);

            currentSystemStatus = newStatus;

            switch(newStatus)
            {
                case ODRIVE_SYSTEM_STATUS_ERROR:
                    StopDriveForErrors();
                    break;
                default:
                    break;
            }
        }

        void WatchdogExpired(float duration)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ODrive Board Watchdog timed out after %f seconds", duration);
            currentSystemErrors.WatchdogFromODriveTimeoutError = true;
            ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_ERROR);
        }

        void StopDriveForErrors()
        {
            sendAxisStateRequest(ODRIVE_AXIS_STATE_Idle);
            SendCanData(ODRIVE_COMMAND_Estop, 0, 0);
            sendSetInputVelocity(0, 0);
        }

        void RequestErrors()
        {
            if(!canDataPublisher->is_activated()) return;
            if(!SendCanData(ODRIVE_COMMAND_GetEncoderError, 0, 0, true))
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to request encoder error");
            if(!SendCanData(ODRIVE_COMMAND_GetMotorError, 0, 0, true))
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to request motor error");
            if(!SendCanData(ODRIVE_COMMAND_GetSensorlessError, 0, 0, true))
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to request sensorless error");
        }

        void startupAxis(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing axis startup");

            switch(currentSystemStatus)
            {
                case ODRIVE_SYSTEM_STATUS_NOT_READY:
                    switch(calibrationType)
                    {
                        case ODRIVE_AXIS_CALIBRATION_None:
                            response->success = sendAxisStateRequest(ODRIVE_AXIS_STATE_ClosedLoopControl);
                            break;
                        case ODRIVE_AXIS_CALIBRATION_EncoderIndex:
                            response->success = sendAxisStateRequest(ODRIVE_AXIS_STATE_EncoderIndexSearch);
                            break;
                        case ODRIVE_AXIS_CALIBRATION_Full:
                            response->success = sendAxisStateRequest(ODRIVE_AXIS_STATE_FullCalibrationSequence);
                            break;
                    }
                    if(response->success)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting axis calibration");
                        ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_CALIBRATING);
                    }
                    else
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Aborting axis startup");
                    break;
                case ODRIVE_SYSTEM_STATUS_IDLE:
                    
                    response->success = sendAxisStateRequest(ODRIVE_AXIS_STATE_ClosedLoopControl);
                    if(response->success)
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting axis in closed loop");
                    else
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to start axis closed loop");
                    break;
                default:
                    response->success = false;
                    response->message = "System status is not correct to start axis";
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Not starting axis due to incorrect status: %d", currentSystemStatus);
                    break;
            }
        }

        void shutdownAxis(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing axis shutdown");
            response->success = sendAxisStateRequest(ODRIVE_AXIS_STATE_Idle);
            if(response->success)
            {
                if(currentSystemStatus == ODRIVE_SYSTEM_STATUS_RUNNING)
                    ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_IDLE);
                else if(currentSystemStatus == ODRIVE_SYSTEM_STATUS_ERROR) ; //Leave it in error
                else
                    ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_NOT_READY);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis is shutting down");
            }
            else
            {
                ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_ERROR);
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to shutdown axis");
            }
        }

        void clearErrors(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            attemptToClearErrors();
            response->success = true;
        }

        void attemptToClearErrors()
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Clearing errors");

            SendCanData(ODRIVE_COMMAND_ClearErrors, 0, 0);

            if((this->now() - lastIncomingMessageTimeFromOdrive).seconds() <
                    MILLISECOND_TO_SECOND(ODRIVE_TO_US_WATCHDOG_INTERVAL))
                currentSystemErrors.WatchdogFromODriveTimeoutError = false;

            currentSystemErrors.IncomingVelocityTimeoutErrorLatch = currentSystemErrors.IncomingVelocityTimeoutError;
        }

        void rebootBoard(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                response->success = sendRebootCommand();
            }

        bool sendRebootCommand()
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending reboot command");

            if(currentSystemStatus != ODRIVE_SYSTEM_STATUS_ERROR)
                ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_NOT_READY);

            return SendCanData(ODRIVE_COMMAND_RebootODrive, 0, 0);
        }

        void incomingVelocityWatchdogElapsed()
        {
            if(currentAxisState == ODRIVE_AXIS_STATE_ClosedLoopControl && lastInputVelocity != 0)
            {
                if((this->now() - lastIncomingVelocityMessage).seconds() > MILLISECOND_TO_SECOND(INCOMING_VELOCITY_WATCHDOG_TIMEOUT))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Incoming Velocity Timer expired.");
                    currentSystemErrors.IncomingVelocityTimeoutError = currentSystemErrors.IncomingVelocityTimeoutErrorLatch = true;
                    ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_ERROR);
                }
            }
        }

        void setInputVelocity(const std::shared_ptr<odrive_interface::srv::InputVelocity::Request> request,
                            std::shared_ptr<odrive_interface::srv::InputVelocity::Response> response)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Received new input velocity request:\n\tVelocity: %f\n\tTorque_FF: %d",
                request->input_velocity, request->torque_feedforward);

            response->success = sendSetInputVelocity(request->input_velocity, request->torque_feedforward);
        }

        bool sendSetInputVelocity(float velocity, int16_t torque_ff)
        {
            if(currentSystemErrors.ErrorsExist())
            {
                if(velocity != 0)
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send new input velocity. Errors exist");
                    return false;
                }
                else
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Errors exist while attempting to send 0 velocity");
            }

            lastIncomingVelocityMessage = this->now();
            lastInputVelocity = velocity;
            currentSystemErrors.IncomingVelocityTimeoutError = false;

            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending new input velocity request:\n\tVelocity: %f\n\tTorque_FF: %d",
                velocity, torque_ff);

            int8_t data[8];

            memcpy(data, &velocity, sizeof(velocity));
            memcpy(data + 4, &torque_ff, sizeof(torque_ff));

            return SendCanData(ODRIVE_COMMAND_SetInputVelocity, data, sizeof(data));
        }

        bool sendAxisStateRequest(odrive_axis_states newState)
        {
            if(currentSystemErrors.ErrorsExist())
            {
                if(newState != ODRIVE_AXIS_STATE_Idle)
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send new axis state. Errors exist");
                    return false;
                }
                else
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Attempting to send idle state while errors exist");
            }

            if(newState == ODRIVE_AXIS_STATE_ClosedLoopControl)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting velocity to 0");

                if(!sendSetInputVelocity(0, 0))
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send velocity. State will not change");
                    return false;
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending new state request: %d", (int32_t) newState);

            int8_t data[8];

            memcpy(data, &newState, sizeof(int32_t));

            return SendCanData(ODRIVE_COMMAND_SetAxisRequestedState, data, sizeof(int32_t));
        }

        bool SendCanData(odrive_commands command, int8_t * data, int8_t dataLen)
        {
            return SendCanData(command, data, dataLen, false);
        }

        bool SendCanData(odrive_commands command, int8_t * data, int8_t dataLen, bool useRTR)
        {
            auto message = can_interface::msg::CanFrame();

            if(dataLen > CAN_DLC_MAX_SIZE)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Requested data DLC is too big");
                return false;
            }

            message.can_id = (axisNumber << 5) | command;
            message.is_error = message.is_extended_id = false;
            message.is_remote_request = useRTR;
            message.dlc = CAN_DLC_MAX_SIZE;
            if(dataLen > 0)
                memcpy(message.data.data(), data, dataLen);

            if(!canDataPublisher->is_activated())
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN publisher is inactive");
                return false;
            }

            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Sending data to CAN");

            canDataPublisher->publish(message);
            return true;
        }

        void canDataReceived(const can_interface::msg::CanFrame & message)
        {
            int32_t cmd_id;

            if(message.can_id >> 5 != (uint32_t) axisNumber)
                return;

            lastIncomingMessageTimeFromOdrive = this->now();

            if(this->get_current_state().id() != 3)     //3 is the "active" lifecycle state
            {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Ignoring data while in current state");
                return;
            }

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
                case ODRIVE_COMMAND_GetMotorError:
                case ODRIVE_COMMAND_GetEncoderError:
                case ODRIVE_COMMAND_GetSensorlessError:
                    UpdateErrors((odrive_commands) cmd_id, message.data);
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
                {
                    ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_ERROR);
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis error changed to: %d", currentSystemErrors.AxisError);
            }

            if(oldAxisState != currentAxisState)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis state changed to: %d", currentAxisState);

                if(currentSystemStatus == ODRIVE_SYSTEM_STATUS_CALIBRATING && currentAxisState == ODRIVE_AXIS_STATE_Idle)
                    sendAxisStateRequest(ODRIVE_AXIS_STATE_ClosedLoopControl);

                if(currentAxisState == ODRIVE_AXIS_STATE_ClosedLoopControl)
                    ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_RUNNING);

                if(currentSystemStatus == ODRIVE_SYSTEM_STATUS_RUNNING && currentAxisState == ODRIVE_AXIS_STATE_Idle)
                    ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_IDLE);
            }
        }

        void UpdateEncoderEstimates(std::array<uint8_t, 8UL> canData) const
        {
            memcpy(&currentPosition, canData.data(), sizeof(currentPosition));
            memcpy(&currentVelocity, canData.data() + 4, sizeof(currentVelocity));
        }

        void UpdateErrors(odrive_commands command, std::array<uint8_t, 8UL> data)
        {
            int64_t errorCode;
            errorCode = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];
            errorCode = (errorCode << 32) | (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
            switch(command)
            {
                case ODRIVE_COMMAND_GetMotorError:
                    currentSystemErrors.MotorError = errorCode;
                    break;
                case ODRIVE_COMMAND_GetEncoderError:
                    currentSystemErrors.EncoderError = errorCode;
                    break;
                case ODRIVE_COMMAND_GetSensorlessError:
                    currentSystemErrors.SensorlessError = errorCode;
                    break;
                default:
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "An illegal message value was passed to update errors: %d", (int32_t) command);
            }

            if(currentSystemErrors.ErrorsExist()) 
            {
                ChangeSystemStatus(ODRIVE_SYSTEM_STATUS_ERROR);
            }
        }

        void SendAxisStatusMessage()
        {
            auto message = odrive_interface::msg::AxisStatus();

            message.axis_status = currentSystemStatus;
            message.current_position = currentPosition;
            message.current_velocity = currentVelocity;

            if(axisStatusPublisher->is_activated())
                axisStatusPublisher->publish(message);
        }

        rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr canDataSubscription;

        rclcpp_lifecycle::LifecyclePublisher<can_interface::msg::CanFrame>::SharedPtr canDataPublisher;

        rclcpp::Service<odrive_interface::srv::InputVelocity>::SharedPtr inputVelocityService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr axisStartupService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr axisShutdownService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clearErrorService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rebootBoardService;

        rclcpp_lifecycle::LifecyclePublisher<odrive_interface::msg::AxisStatus>::SharedPtr axisStatusPublisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr watchdogTimerMonitorPublisher;

        rclcpp::TimerBase::SharedPtr statusPublishTimer;
        rclcpp::TimerBase::SharedPtr maintenanceTimer;
        rclcpp::TimerBase::SharedPtr errorRequestTimer;
        rclcpp::TimerBase::SharedPtr incomingVelocityTimeoutTimer;

        rclcpp::Time lastIncomingMessageTimeFromOdrive;
        rclcpp::Time lastIncomingVelocityMessage;

        int32_t axisNumber;
        int32_t calibrationType;
        bool publishDebugMessages;
        mutable odrive_axis_states currentAxisState;
        mutable odrive_system_status currentSystemStatus;
        mutable float currentPosition;
        mutable float currentVelocity;
        float lastInputVelocity;
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