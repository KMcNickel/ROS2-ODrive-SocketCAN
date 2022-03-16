#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "can_interface/msg/can_frame.hpp"
#include "odrive_interface/msg/heartbeat.hpp"

using std::placeholders::_1;

class ODrive : public rclcpp::Node
{
    public:
        ODrive()
        : Node("ODrive")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Up");

            this->declare_parameter<std::int32_t>("axis_number", 0);

            this->get_parameter("axis_number", axisNumber);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using Axis %d", axisNumber);

            createSubscribers();

            createPublishers();
        }

        void canDataReceived(const can_interface::msg::CanFrame & message) const
        {
            int32_t cmd_id;

            if(message.can_id >> 5 != axisNumber)
                return;

            cmd_id = message.can_id & 0b00011111;

            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Processing message of command: %d", cmd_id);

            switch((odrive_commands) cmd_id)
            {
                case ODRIVE_COMMAND_Heartbeat:
                    UpdateHeartbeat(message.data);
                    break;
                default:
                    break;
            }
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

        /*
        Example send data:
        auto message = can_interface::msg::CanFrame();
        ## fill in message struct ##
        std::copy(std::begin(source), std::end(source), std::begin(message.data));
        canDataPublisher->publish(message);
        */

        ~ODrive()
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting Down");
            //Do Stuff
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown complete. Goodbye!");
        }

    private:
        void createPublishers()
        {
            canDataPublisher = this->create_publisher<can_interface::msg::CanFrame>("odrivecan/sender/data", 10);
            heartbeatPublisher = this->create_publisher<odrive_interface::msg::Heartbeat>("odrive/status/heartbeat", 10);
        }

        void createSubscribers()
        {
            canDataSubscription = this->create_subscription<can_interface::msg::CanFrame>(
                "odrivecan/receiver/data", 50, std::bind(&ODrive::canDataReceived, this, _1));
        }

        void UpdateHeartbeat(std::array<int8_t, 8UL> canData) const
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

        rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr canDataSubscription;
        rclcpp::Publisher<can_interface::msg::CanFrame>::SharedPtr canDataPublisher;
        rclcpp::Publisher<odrive_interface::msg::Heartbeat>::SharedPtr heartbeatPublisher;

        int32_t axisNumber;
        mutable odrive_axis_states currentAxisState;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODrive>());
    rclcpp::shutdown();
    return 0;
}