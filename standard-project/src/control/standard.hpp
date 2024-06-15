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
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"


#include "tap/algorithms/smooth_pid.hpp"


// using name



#include "control/flywheel/fly_wheel_subsystem.hpp"
#include "control/flywheel/fly_wheel_shoot_command.hpp"
#include "control/control_operator_interface.hpp"

class Drivers;

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



    control::ControlOperatorInterface m_ControlOperatorInterface;

    control::flyWheel::FlyWheelSubsystem m_FlyWheel;

    control::flyWheel::flyWheelCommand m_FlyWheelCommand;
};  
}  // namespace control
