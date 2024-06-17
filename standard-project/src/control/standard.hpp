/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-edu.
 *
 * aruw-edu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-edu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-edu.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"

#include "control/agitator/velocity_agitator_subsystem.hpp"

#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"

#include "control/chassis/mecanum_drive_command.hpp"

#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"


#include "tap/algorithms/smooth_pid.hpp"


// using name



#include "control/flywheel/fly_wheel_subsystem.hpp"
#include "control/flywheel/fly_wheel_shoot_command.hpp"
#include "control/control_operator_interface.hpp"
#include "turret/turret_super_structure/standard_turret_subsystem.hpp"
#include "turret/turret_control/turret_user_control_command.hpp"
#include "turret/turret_components/chassis_frame_turret_controller.hpp"
#include "turret/turret_components/yaw_turret_motor.hpp"

#include "control/chassis/turret_orientated_drive_command.hpp"



class Drivers;


using namespace control::turret;
using namespace control::turret::user;
using namespace control::turret::algorithms;
namespace control
{
class Robot
{
public:
    Robot(src::Drivers &drivers);

    void initSubsystemCommands();
    

private:
    void initializeSubsystems();
    void registerSoldierSubsystems();
    void setDefaultSoldierCommands();
    void startSoldierCommands();
    void registerSoldierIoMappings();

    src::Drivers &drivers;

    control::chassis::ChassisSubsystem m_ChassisSubsystem;

    // control::chassis::MecanumDriveCommand m_MecanumDriveCommand;
    
    
    control::agitator::VelocityAgitatorSubsystemConfig agitatorSubsystemConfig;
    tap::algorithms::SmoothPidConfig agitatorVelocityPidConfig;
    control::agitator::VelocityAgitatorSubsystem agitatorSubsystem;
    tap::control::setpoint::MoveIntegralCommand::Config rotateAgitatorCommandConfig;
    tap::control::setpoint::MoveIntegralCommand rotateAgitatorCommand;
    tap::control::setpoint::UnjamIntegralCommand::Config unjamAgitatorCommandConfig;
    tap::control::setpoint::UnjamIntegralCommand unjamAgitatorCommand;
    tap::control::setpoint::MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitatorCommand;
    tap::control::HoldCommandMapping leftMousePressed;
    tap::control::HoldRepeatCommandMapping rightMousePressed;
    tap::control::HoldRepeatCommandMapping leftSwitchUp;



    control::ControlOperatorInterface m_ControlOperatorInterface;

    control::flyWheel::FlyWheelSubsystem m_FlyWheel;

    control::flyWheel::flyWheelCommand m_FlyWheelCommand;
    tap::motor::DjiMotor pitchMotor;
    TurretMotor turretPitchMotor;
    tap::motor::DjiMotor yawMotor;
    TurretMCBCGryo turretGyro;
    YawTurretMotor turretYawMotor;
    StandardTurretSubsystem turret;
    ChassisFrameYawTurretController yawController;
    ChassisFramePitchTurretController pitchController;
    TurretUserControlCommand turretUserControlCommand;

    control::chassis::ChassisTurretDriveCommand turretOrientedDriveCommand;

    

    


    
    
};  
}  // namespace control
