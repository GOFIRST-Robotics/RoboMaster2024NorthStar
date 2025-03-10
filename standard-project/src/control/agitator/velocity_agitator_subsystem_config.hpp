/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VELOCITY_AGITATOR_SUBSYSTEM_CONFIG_HPP_
#define VELOCITY_AGITATOR_SUBSYSTEM_CONFIG_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

namespace control::agitator
{
/**
 * @param[in] jammingDistance jamming timer counts down when distance between
 *      setpoint and current angle is > `jammingDistance` and resets timer when
 *      distance is <= `jammingDistance`.
 * @param[in] jammingTime how long the jamming timer is. Once this timer finishes
 *      the subsystem is considered jammed
 * @param[in] jamLogicEnabled whether or not to enable jam detection
 *
 * Configuration struct for the velocity agitator subsystem
 */
struct VelocityAgitatorSubsystemConfig
{
    /// Motor gear ratio, so we use shaft angle rather than encoder angle.
    float gearRatio;
    /// The motor ID for this motor.
    tap::motor::MotorId agitatorMotorId;
    /// The motor CAN bus for this motor.
    tap::can::CanBus agitatorCanBusId;
    /// If `true` positive rotation is clockwise when looking at the motor shaft opposite the motor.
    /// Counterclockwise if false.
    bool isAgitatorInverted;
    /// Difference between target and setpoint velocity.
    float jammingVelocityDifference;
    /// How long the jamming timer is. If the target/setpoint difference is > the
    /// jammingVelocityDifference for jammingTime the agitator is considered to be jammed.
    uint32_t jammingTime;
    /// A flag which determines whether or not jamming detection is enabled. `true` means enabled,
    /// `false` means disabled.
    bool jamLogicEnabled;
    /// Velocity PID feed forward term. Scaling factor that converts desired velocity to desired
    /// motor output. When using the M308 or the M2006, motor velocity -> motor current is mostly
    /// linear since these motors take a desired current as a command. When using a motor that is
    /// controlled by sending voltage commands, this term should be 0.
    float velocityPIDFeedForwardGain;
};
}  // namespace aruwsrc::agitator

#endif  // VELOCITY_AGITATOR_SUBSYSTEM_CONFIG_HPP_
