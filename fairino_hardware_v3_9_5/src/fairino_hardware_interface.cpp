#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware
{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& sysinfo)
{
    if (hardware_interface::SystemInterface::on_init(sysinfo) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = sysinfo;  // info_ 是父类中定义的变量

    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {
        // 指令部分
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "Joint '%s' has command interface '%s'. Expected '%s'.",
                joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 关节状态部分
        if (joint.state_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "Joint '%s' has %zu state interfaces found. 1 expected.",
                joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "Joint '%s' has state interface '%s'. Expected '%s'.",
                joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
FairinoHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &_jnt_position_state[i]));
    }

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
FairinoHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &_jnt_position_command[i]));
    }

    return command_interfaces;
}


hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;

    using namespace std::chrono_literals;

    RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Starting ...please wait...");

    _ptr_robot = std::make_unique<FRRobot>();

    for (int i = 0; i < 6; ++i)
    {
        _jnt_position_command[i] = 0.0;
        _jnt_velocity_command[i] = 0.0;
        _jnt_torque_command[i] = 0.0;

        _jnt_position_state[i] = 0.0;
        _jnt_velocity_state[i] = 0.0;
        _jnt_torque_state[i] = 0.0;
    }

    // 默认是位置控制: 0-position, 1-torque, 2-velocity
    _control_mode = 0;

    // 建立 XMLRPC 连接
    error_t returncode = _ptr_robot->RPC(_controller_ip.c_str());
    rclcpp::sleep_for(200ms);

    if (returncode != 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "机械臂SDK连接失败！请检查IP、网络或端口是否被占用。错误码:%d",
            returncode);
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "机械臂SDK连接成功！");

    // 读取当前真实关节角度
    JointPos jntpos;
    returncode = _ptr_robot->GetActualJointPosDegree(0, &jntpos);

    /*
     * 获取反馈位置后同步到指令位置以维持当前状态。
     * 如果读取失败，则不能激活硬件，否则初始指令可能与真实位置不一致，存在安全风险。
     */
    if (returncode != 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "读取初始关节角度错误，硬件无法启动！错误码:%d",
            returncode);
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (int j = 0; j < 6; ++j)
    {
        _jnt_position_state[j] = jntpos.jPos[j] / 180.0 * M_PI;
        _jnt_position_command[j] = _jnt_position_state[j];
    }

    RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "初始指令位置(rad): %f,%f,%f,%f,%f,%f",
        _jnt_position_command[0],
        _jnt_position_command[1],
        _jnt_position_command[2],
        _jnt_position_command[3],
        _jnt_position_command[4],
        _jnt_position_command[5]);

    /*
     * ServoJ requires the robot to be in servo motion mode.
     * Without ServoMoveStart(), ServoJ can block and cause controller_manager overruns.
     */
    returncode = _ptr_robot->ResetAllError();
    if (returncode != 0)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "ResetAllError returned code: %d",
            returncode);
    }

    returncode = _ptr_robot->RobotEnable(1);
    if (returncode != 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "RobotEnable(1) failed with code: %d",
            returncode);
        return hardware_interface::CallbackReturn::ERROR;
    }

    returncode = _ptr_robot->StopMotion();
    if (returncode != 0)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "StopMotion returned code: %d",
            returncode);
    }

    returncode = _ptr_robot->ServoMoveEnd();
    if (returncode != 0)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "ServoMoveEnd returned code: %d",
            returncode);
    }

    returncode = _ptr_robot->ServoMoveStart();
    if (returncode != 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "ServoMoveStart failed with code: %d",
            returncode);
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "机械臂硬件启动成功，ServoJ模式已启用！");

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Stopping ...please wait...");

    if (_ptr_robot)
    {
        error_t returncode = _ptr_robot->ServoMoveEnd();
        if (returncode != 0)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "ServoMoveEnd returned code: %d",
                returncode);
        }

        returncode = _ptr_robot->StopMotion();
        if (returncode != 0)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "StopMotion returned code: %d",
                returncode);
        }

        _ptr_robot->CloseRPC();
        _ptr_robot.reset();
    }

    RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type FairinoHardwareInterface::read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period)
{
    (void)time;
    (void)period;

    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    JointPos state_data;
    error_t returncode = _ptr_robot->GetActualJointPosDegree(1, &state_data);

    if (returncode == 0)
    {
        for (int i = 0; i < 6; ++i)
        {
            // MoveIt / ros2_control use radians
            _jnt_position_state[i] = state_data.jPos[i] / 180.0 * M_PI;
        }

        return hardware_interface::return_type::OK;
    }

    RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("FairinoHardwareInterface"),
        steady_clock,
        1000,
        "GetActualJointPosDegree failed with code: %d. Keeping last valid state.",
        returncode);

    // Mantener último estado válido evita tirar todo ros2_control por una lectura puntual fallida.
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type FairinoHardwareInterface::write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period)
{
    (void)time;
    (void)period;

    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    if (_control_mode == 0)  // 位置控制模式
    {
        // Important: STL ranges are [first, last), so use +6 to check all 6 joints.
        if (std::any_of(
                &_jnt_position_command[0],
                &_jnt_position_command[0] + 6,
                [](double c) { return !std::isfinite(c); }))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "Invalid joint position command detected.");
            return hardware_interface::return_type::ERROR;
        }

        JointPos cmd;
        ExaxisPos extcmd{0, 0, 0, 0};

        for (int j = 0; j < 6; ++j)
        {
            // Fairino SDK expects degrees
            cmd.jPos[j] = _jnt_position_command[j] / M_PI * 180.0;
        }

        int returncode = _ptr_robot->ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);

        if (returncode != 0)
        {
            RCLCPP_WARN_THROTTLE(
                rclcpp::get_logger("FairinoHardwareInterface"),
                steady_clock,
                1000,
                "ServoJ指令下发错误,错误码:%d",
                returncode);
        }
    }
    else if (_control_mode == 1)  // 扭矩控制模式，预留
    {
        if (std::any_of(
                &_jnt_torque_command[0],
                &_jnt_torque_command[0] + 6,
                [](double c) { return !std::isfinite(c); }))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "Invalid joint torque command detected.");
            return hardware_interface::return_type::ERROR;
        }

        // Torque mode not implemented yet.
        return hardware_interface::return_type::OK;
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "指令发送错误:未识别当前所处控制模式");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace fairino_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    fairino_hardware::FairinoHardwareInterface,
    hardware_interface::SystemInterface)
