
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "osc_interfaces/msg/device_status.hpp"
#include "osc_interfaces/msg/motor_state.hpp"
#include "osc_interfaces/msg/motors_states.hpp"
#include "osc_utils/simple_publisher.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"

namespace odrive_ros2_control {

class Axis;

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);
    void set_axis_command_mode(const Axis& axis);
    std::string get_error_string(uint32_t error_code);
    osc_interfaces::msg::MotorsStates generate_motors_states_msg(const rclcpp::Time& timestamp);
    std::string decode(uint32_t code, const std::unordered_map<uint32_t, std::string>& map);

    bool active_;
    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
    rclcpp::Time timestamp_pub_;
    std::shared_ptr<SimplePublisher<osc_interfaces::msg::MotorsStates>> pub_;
    double pub_period_s_;
    double heartbeat_timeout_s_;
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id) : can_intf_(can_intf), node_id_(node_id) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    void on_can_msg();

    SocketCanIntf* can_intf_;
    uint32_t node_id_;

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0f; // [rad]
    double vel_setpoint_ = 0.0f; // [rad/s]
    double torque_setpoint_ = 0.0f; // [Nm]

    // State (ODrives => ros2_control)
    // rclcpp::Time encoder_estimates_timestamp_;
    // uint32_t axis_error_ = 0;
    // uint8_t axis_state_ = 0;
    // uint8_t procedure_result_ = 0;
    // uint8_t trajectory_done_flag_ = 0;
    double pos_estimate_ = NAN; // [rad]
    double vel_estimate_ = NAN; // [rad/s]
    // double iq_setpoint_ = NAN;
    // double iq_measured_ = NAN;
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    // uint32_t active_errors_ = 0;
    // uint32_t disarm_reason_ = 0;
    // double fet_temperature_ = NAN;
    double motor_temperature_ = NAN;
    double bus_voltage_ = NAN;
    double bus_current_ = NAN;
    double direction_multiplier_ = 1.0;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    uint32_t error_code_ = ODRIVE_ERROR_NONE;
    uint32_t procedure_result_ = PROCEDURE_RESULT_SUCCESS;
    uint32_t axis_state_ = AXIS_STATE_IDLE;

    rclcpp::Time timestamp_heartbeat_{rclcpp::Time(0, 0, RCL_ROS_TIME)};

    template <typename T>
    void send(const T& msg) const {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        can_intf_->send_can_frame(frame);
    }
};

const std::unordered_map<uint32_t, std::string> ODRIVE_ERROR_MAP = {
    {ODRIVE_ERROR_NONE, "OK"},
    {ODRIVE_ERROR_INITIALIZING, "Initializing"},
    {ODRIVE_ERROR_SYSTEM_LEVEL, "System Level Error"},
    {ODRIVE_ERROR_TIMING_ERROR, "Timing Error"},
    {ODRIVE_ERROR_MISSING_ESTIMATE, "Missing Estimate"},
    {ODRIVE_ERROR_BAD_CONFIG, "Bad Config"},
    {ODRIVE_ERROR_DRV_FAULT, "DRV Fault"},
    {ODRIVE_ERROR_MISSING_INPUT, "Missing Input"},
    {ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE, "DC Bus Over Voltage"},
    {ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE, "DC Bus Under Voltage"},
    {ODRIVE_ERROR_DC_BUS_OVER_CURRENT, "DC Bus Over Current"},
    {ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT, "DC Bus Over Regen Current"},
    {ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION, "Current Limit Violation"},
    {ODRIVE_ERROR_MOTOR_OVER_TEMP, "Motor Over Temperature"},
    {ODRIVE_ERROR_INVERTER_OVER_TEMP, "Inverter Over Temperature"},
    {ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION, "Velocity Limit Violation"},
    {ODRIVE_ERROR_POSITION_LIMIT_VIOLATION, "Position Limit Violation"},
    {ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED, "Watchdog Timer Expired"},
    {ODRIVE_ERROR_ESTOP_REQUESTED, "E-Stop Requested"},
    {ODRIVE_ERROR_SPINOUT_DETECTED, "Spinout Detected"},
    {ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED, "Brake Resistor Disarmed"},
    {ODRIVE_ERROR_THERMISTOR_DISCONNECTED, "Thermistor Disconnected"},
    {ODRIVE_ERROR_CALIBRATION_ERROR, "Calibration Error"}
};

const std::unordered_map<uint32_t, std::string> ODRIVE_PROCEDURE_RESULT_MAP = {
    {PROCEDURE_RESULT_SUCCESS, "Success"},
    {PROCEDURE_RESULT_BUSY, "Busy"},
    {PROCEDURE_RESULT_CANCELLED, "Cancelled"},
    {PROCEDURE_RESULT_DISARMED, "Disarmed"},
    {PROCEDURE_RESULT_NO_RESPONSE, "No Response"},
    {PROCEDURE_RESULT_POLE_PAIR_CPR_MISMATCH, "Pole Pair CPR Mismatch"},
    {PROCEDURE_RESULT_PHASE_RESISTANCE_OUT_OF_RANGE, "Phase Resistance Out of Range"},
    {PROCEDURE_RESULT_PHASE_INDUCTANCE_OUT_OF_RANGE, "Phase Inductance Out of Range"},
    {PROCEDURE_RESULT_UNBALANCED_PHASES, "Unbalanced Phases"},
    {PROCEDURE_RESULT_INVALID_MOTOR_TYPE, "Invalid Motor Type"},
    {PROCEDURE_RESULT_ILLEGAL_HALL_STATE, "Illegal Hall State"},
    {PROCEDURE_RESULT_TIMEOUT, "Timeout"},
    {PROCEDURE_RESULT_HOMING_WITHOUT_ENDSTOP, "Homing Without Endstop"},
    {PROCEDURE_RESULT_INVALID_STATE, "Invalid State"},
    {PROCEDURE_RESULT_NOT_CALIBRATED, "Not Calibrated"},
    {PROCEDURE_RESULT_NOT_CONVERGING, "Not Converging"}
};

const std::unordered_map<uint32_t, std::string> ODRIVE_AXIS_STATE_MAP = {
    {AXIS_STATE_UNDEFINED, "Undefined"},
    {AXIS_STATE_IDLE, "Idle"},
    {AXIS_STATE_STARTUP_SEQUENCE, "Startup Sequence"},
    {AXIS_STATE_FULL_CALIBRATION_SEQUENCE, "Full Calibration Sequence"},
    {AXIS_STATE_MOTOR_CALIBRATION, "Motor Calibration"},
    {AXIS_STATE_ENCODER_INDEX_SEARCH, "Encoder Index Search"},
    {AXIS_STATE_ENCODER_OFFSET_CALIBRATION, "Encoder Offset Calibration"},
    {AXIS_STATE_CLOSED_LOOP_CONTROL, "Closed Loop Control"},
    {AXIS_STATE_LOCKIN_SPIN, "Lockin Spin"},
    {AXIS_STATE_ENCODER_DIR_FIND, "Encoder Direction Find"},
    {AXIS_STATE_HOMING, "Homing"},
    {AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION, "Encoder Hall Polarity Calibration"},
    {AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION, "Encoder Hall Phase Calibration"},
    {AXIS_STATE_ANTICOGGING_CALIBRATION, "Anticogging Calibration"}
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    pub_period_s_ = 1 / std::stod(info.hardware_parameters.at("motor_state_pub_rate_hz"));
    can_intf_name_ = info_.hardware_parameters["can"];

    for (auto& joint : info_.joints) {
        try {
            axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "Invalid or missing 'node_id' parameter for joint %s: %s",
                joint.name.c_str(),
                e.what()
            );
            return CallbackReturn::ERROR;
        }

        auto& axis = axes_.back();

        auto rotation_param_it = joint.parameters.find("inverse_rotation");
        if (rotation_param_it != joint.parameters.end()) {
            const auto& rotation_param = rotation_param_it->second;
            if (rotation_param == "true" || rotation_param == "True") {
                axis.direction_multiplier_ = -1.0;
            } else if (rotation_param == "false" || rotation_param == "False") {
                axis.direction_multiplier_ = 1.0;
            } else {
                RCLCPP_ERROR(
                    rclcpp::get_logger("ODriveHardwareInterface"),
                    "Invalid 'inverse_rotation' parameter for joint '%s': %s. Expected 'true' or 'false'.",
                    joint.name.c_str(),
                    rotation_param.c_str()
                );
                return CallbackReturn::ERROR;
            }
        } else {
            // Default value if inverse_rotation is not specified
            axis.direction_multiplier_ = 1.0;
        }
    }

    pub_ = std::make_shared<SimplePublisher<osc_interfaces::msg::MotorsStates>>("odrive_hw", "odrive_state");
    heartbeat_timeout_s_ = std::stod(info.hardware_parameters.at("heartbeat_timeout_period_s"));
    timestamp_pub_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ODriveHardwareInterface"),
            "Failed to initialize SocketCAN on %s",
            can_intf_name_.c_str()
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");

    // This can be called several seconds before the controller finishes starting.
    // Therefore we enable the ODrives only in perform_command_mode_switch().

    active_ = true;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

    active_ = false;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_EFFORT,
                &axes_[i].torque_target_
            )
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &axes_[i].vel_estimate_
            )
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &axes_[i].pos_estimate_
            )
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_EFFORT,
                &axes_[i].torque_setpoint_
            )
        );
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &axes_[i].vel_setpoint_
            )
        );
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &axes_[i].pos_setpoint_
            )
        );
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}
        };

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            set_axis_command_mode(axis);
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    timestamp_ = timestamp;

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }
    if ((timestamp_.seconds() - timestamp_pub_.seconds()) > pub_period_s_) {
        osc_interfaces::msg::MotorsStates msg = generate_motors_states_msg(timestamp);
        pub_->publishData(msg);
        timestamp_pub_ = timestamp_;
    }
    return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (auto& axis : axes_) {
        // Send the CAN message that fits the set of enabled setpoints
        if (axis.pos_input_enabled_) {
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = axis.direction_multiplier_ * axis.pos_setpoint_ / (2 * M_PI);
            msg.Vel_FF = axis.vel_input_enabled_ ? (axis.direction_multiplier_ * axis.vel_setpoint_ / (2 * M_PI))
                                                 : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? (axis.direction_multiplier_ * axis.torque_setpoint_) : 0.0f;
            axis.send(msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel = axis.direction_multiplier_ * axis.vel_setpoint_ / (2 * M_PI);
            msg.Input_Torque_FF = axis.torque_input_enabled_ ? (axis.direction_multiplier_ * axis.torque_setpoint_)
                                                             : 0.0f;
            axis.send(msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = axis.torque_setpoint_;
            axis.send(msg);
        } else {
            // no control enabled - don't send any setpoint
        }
    }

    return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void ODriveHardwareInterface::set_axis_command_mode(const Axis& axis) {
    if (!active_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Interface inactive. Setting axis to idle.");
        Set_Axis_State_msg_t idle_msg;
        idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(idle_msg);
        return;
    }

    Set_Controller_Mode_msg_t control_msg;
    Clear_Errors_msg_t clear_error_msg;
    Set_Axis_State_msg_t state_msg;

    clear_error_msg.Identify = 0;
    control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
    state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

    if (axis.pos_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to position control.");
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
    } else if (axis.vel_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to velocity control.");
        control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    } else if (axis.torque_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to torque control.");
        control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "No control mode specified. Setting to idle.");
        state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(state_msg);
        return;
    }

    axis.send(control_msg);
    axis.send(clear_error_msg);
    axis.send(state_msg);
}

osc_interfaces::msg::MotorsStates ODriveHardwareInterface::generate_motors_states_msg(const rclcpp::Time& timestamp) {
    osc_interfaces::msg::MotorsStates msg;
    msg.header.stamp = timestamp;
    msg.motors_type = "Odrive hardware";

    for (auto& axis : axes_) {
        osc_interfaces::msg::MotorState motor_msg;
        motor_msg.uid = static_cast<uint8_t>(axis.node_id_);

        if (axis.timestamp_heartbeat_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
            motor_msg.motor_status = osc_interfaces::msg::DeviceStatus::STATUS_UNREACHABLE;
            msg.data.push_back(motor_msg);
            continue;
        } else if (timestamp.seconds() - axis.timestamp_heartbeat_.seconds() > heartbeat_timeout_s_) {
            motor_msg.motor_status = osc_interfaces::msg::DeviceStatus::STATUS_COMMUNICATION_TIMEOUT;
            double timeout_delay = timestamp.seconds() - axis.timestamp_heartbeat_.seconds();
            motor_msg.status_detail = std::to_string(timeout_delay) + " since last heartbeat";
            msg.data.push_back(motor_msg);
            continue;
        }

        motor_msg.bus_voltage_v = axis.bus_voltage_;
        motor_msg.bus_current_ma = axis.bus_current_ * 1000;
        motor_msg.computed_torque_nm = axis.torque_estimate_;
        motor_msg.temperature_c = axis.motor_temperature_;

        if (axis.error_code_ != ODRIVE_ERROR_NONE) {
            motor_msg.motor_status = osc_interfaces::msg::DeviceStatus::STATUS_ERROR;
            motor_msg.motor_control_mode = osc_interfaces::msg::MotorState::CONTROL_MODE_IDLE;
            motor_msg.command_setpoint = 0.0;
            motor_msg.command_actual = 0.0;
            motor_msg.status_detail  = decode(axis.error_code_, ODRIVE_ERROR_MAP);
        } else if (axis.procedure_result_ != PROCEDURE_RESULT_SUCCESS) {
            motor_msg.motor_status = osc_interfaces::msg::DeviceStatus::STATUS_ERROR;
            motor_msg.motor_control_mode = osc_interfaces::msg::MotorState::CONTROL_MODE_IDLE;
            motor_msg.command_setpoint = 0.0;
            motor_msg.command_actual = 0.0;
            motor_msg.status_detail = decode(axis.procedure_result_, ODRIVE_PROCEDURE_RESULT_MAP);
        } else if (axis.axis_state_ != AXIS_STATE_CLOSED_LOOP_CONTROL) {
            motor_msg.motor_status = osc_interfaces::msg::DeviceStatus::STATUS_IDLE;
            motor_msg.motor_control_mode = osc_interfaces::msg::MotorState::CONTROL_MODE_IDLE;
            motor_msg.command_setpoint = 0.0;
            motor_msg.command_actual = 0.0;
            motor_msg.status_detail = decode(axis.axis_state_, ODRIVE_AXIS_STATE_MAP);
        } else if (axis.pos_input_enabled_) {
            motor_msg.motor_status = osc_interfaces::msg::DeviceStatus::STATUS_RUNNING;
            motor_msg.motor_control_mode = osc_interfaces::msg::MotorState::CONTROL_MODE_POSITION;
            motor_msg.command_setpoint = axis.pos_setpoint_;
            motor_msg.command_actual = axis.pos_estimate_;
        } else if (axis.vel_input_enabled_) {
            motor_msg.motor_status = osc_interfaces::msg::DeviceStatus::STATUS_RUNNING;
            motor_msg.motor_control_mode = osc_interfaces::msg::MotorState::CONTROL_MODE_VELOCITY;
            motor_msg.command_setpoint = axis.vel_setpoint_;
            motor_msg.command_actual = axis.vel_estimate_;
        } else if (axis.torque_input_enabled_) {
            motor_msg.motor_status = osc_interfaces::msg::DeviceStatus::STATUS_RUNNING;
            motor_msg.motor_control_mode = osc_interfaces::msg::MotorState::CONTROL_MODE_TORQUE;
            motor_msg.command_setpoint = axis.torque_setpoint_;
            motor_msg.command_actual = axis.torque_estimate_;
        }
        msg.data.push_back(motor_msg);
    }

    return msg;
}

std::string ODriveHardwareInterface::decode(uint32_t code, const std::unordered_map<uint32_t, std::string>& map)
{
    std::string description;

    for (const auto& pair : map)
    {
        if (code & pair.first)
        {
            description += pair.second + ", ";
        }
    }

    if (!description.empty())
    {
        description.erase(description.length() - 2);
    }

    return description;
}

void Axis::on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI) * direction_multiplier_;
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI) * direction_multiplier_;
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target * direction_multiplier_;
                torque_estimate_ = msg.Torque_Estimate * direction_multiplier_;
            }
        } break;
        case Get_Bus_Voltage_Current_msg_t::cmd_id: {
            if (Get_Bus_Voltage_Current_msg_t msg; try_decode(msg)) {
                bus_voltage_ = msg.Bus_Voltage;
                bus_current_ = msg.Bus_Current;
            }
        } break;
        case Get_Temperature_msg_t::cmd_id: {
            if (Get_Temperature_msg_t msg; try_decode(msg)) {
                motor_temperature_ = msg.Motor_Temperature;
            }
        } break;
        case Get_Error_msg_t::cmd_id: {
            if (Get_Error_msg_t msg; try_decode(msg)) {
                error_code_ = msg.Active_Errors;
            }
        } break;
        case Heartbeat_msg_t::cmd_id: {
            if (Heartbeat_msg_t msg; try_decode(msg)) {
                timestamp_heartbeat_ = timestamp;
                procedure_result_ = msg.Procedure_Result;
                axis_state_ = msg.Axis_State;
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
