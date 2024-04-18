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

#include "standard.hpp"

#include "tap/util_macros.hpp"

#include "drivers.hpp"

using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;

namespace control
{
Robot::Robot(src::Drivers &drivers) 
    : drivers(drivers),
    m_FlyWheel(
        drivers,
        &drivers.pwm,
        tap::gpio::Pwm::C1,
        tap::gpio::Pwm::C2),
    m_ControlOperatorInterface(drivers.remote),
    m_FlyWheelCommand(m_FlyWheel, m_ControlOperatorInterface)
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
    m_FlyWheel.initialize();
}

void Robot::registerSoldierSubsystems()
{
    drivers.commandScheduler.registerSubsystem(&m_FlyWheel);
}

void Robot::setDefaultSoldierCommands()
{
    m_FlyWheel.setDefaultCommand(&m_FlyWheelCommand);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{
}   
}  // namespace control
