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
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "turret/turret_super_structure/standard_turret_subsystem.hpp"
#include "turret/turret_control/turret_user_control_command.hpp"
#include "turret/turret_components/chassis_frame_turret_controller.hpp"



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
    ControlOperatorInterface controlOperatorInterface;
    tap::motor::DjiMotor pitchMotor;
    TurretMotor turretPitchMotor;
    tap::motor::DjiMotor yawMotor;
    TurretMotor turretYawMotor;
    TurretMCBCGryo turretGyro;
    StandardTurretSubsystem turret;
    ChassisFrameYawTurretController yawController;
    ChassisFramePitchTurretController pitchController;
    TurretUserControlCommand turretUserControlCommand;

    


    
    
};
}  // namespace control
