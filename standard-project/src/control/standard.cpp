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



#include "drivers.hpp"

#include "standard.hpp"

#include "tap/util_macros.hpp"
#include "turret/turret_constants/standard_turret_constants.hpp"


using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;
// using namespace control::turret;

namespace control
{
Robot::Robot(src::Drivers &drivers) 
    : drivers(
        drivers), 
    controlOperatorInterface(
        drivers.remote),
    pitchMotor(
        &drivers, 
        MotorId::MOTOR7, 
        CanBus::CAN_BUS1, 
        false, "pitchMotor"),
    turretPitchMotor(
        &pitchMotor,
        PITCH_MOTOR_CONFIG),
    yawMotor(
        &drivers, 
        MotorId::MOTOR8, 
        CanBus::CAN_BUS1, 
        false, "YawMotor"),
    turretYawMotor(
        &yawMotor,
        YAW_MOTOR_CONFIG
    ),
    turretGyro(
        &drivers),
    turret(
        &drivers,
        &pitchMotor,
        &yawMotor, 
        PITCH_MOTOR_CONFIG,
        YAW_MOTOR_CONFIG, 
        turretGyro),
    yawController(
        turretYawMotor, 
        YAW_PID_CONFIG),
    pitchController(
        turretPitchMotor, 
        PITCH_PID_CONFIG),
    turretUserControlCommand(
        &drivers,
        controlOperatorInterface,
        &turret, 
        &yawController,
        &pitchController,
        USER_YAW_INPUT_SCALAR,
        USER_PITCH_INPUT_SCALAR,
        0
    )
{
    
}

void Robot::initSubsystemCommands()
{
    initializeSubsystems();
    registerSoldierSubsystems();
    setDefaultSoldierCommands();
    startSoldierCommands();
    registerSoldierIoMappings();
}

void Robot::initializeSubsystems()
{
    turret.initialize();
}

void Robot::registerSoldierSubsystems()
{
    drivers.commandScheduler.registerSubsystem(&turret);
}

void Robot::setDefaultSoldierCommands()
{
    turret.setDefaultCommand(&turretUserControlCommand);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{

}   
}  // namespace control
