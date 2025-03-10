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
using tap::control::setpoint::IntegrableSetpointSubsystem;
using tap::control::setpoint::MoveIntegralCommand;
using tap::control::setpoint::UnjamIntegralCommand;
using tap::control::setpoint::MoveUnjamIntegralComprisedCommand;

// using namespace control::turret;

namespace control
{
Robot::Robot(src::Drivers &drivers) 
    : drivers(drivers),
      m_ChassisSubsystem(
          drivers,
          chassis::ChassisConfig{
              .leftFrontId = MotorId::MOTOR2,
              .leftBackId = MotorId::MOTOR3,
              .rightBackId = MotorId::MOTOR4,
              .rightFrontId = MotorId::MOTOR1,
              .canBus = CanBus::CAN_BUS1,
              .wheelVelocityPidConfig = modm::Pid<float>::Parameter(10, 0, 0, 0, 16'000),
          }),
      agitatorSubsystemConfig{
        .gearRatio = 36.0f,
        .agitatorMotorId = tap::motor::MOTOR7,
        .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
        .isAgitatorInverted = false,
        .jammingVelocityDifference = M_TWOPI,
        .jammingTime = 100,
        .jamLogicEnabled = true,
        .velocityPIDFeedForwardGain = 500.0f / M_TWOPI
    },
    agitatorVelocityPidConfig {
    .kp = 5'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 10000, //DjiMotor::MAX_OUTPUT_C610,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    },
    agitatorSubsystem(
        &drivers, 
        agitatorVelocityPidConfig, 
        agitatorSubsystemConfig),
    rotateAgitatorCommandConfig{
        .targetIntegralChange = M_TWOPI / 10.0f,
        .desiredSetpoint = M_TWOPI,
        .integralSetpointTolerance = 0,
    },
    rotateAgitatorCommand(
        agitatorSubsystem,
        rotateAgitatorCommandConfig),
    unjamAgitatorCommandConfig{
        .targetUnjamIntegralChange = 0.6f * (M_TWOPI / 8), //AGITATOR_NUM_POCKETS),
        .unjamSetpoint = 0.15f * 20 * (M_TWOPI / 8),
        /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
        /// seconds.Convert to ms, Add 100 ms extra tolerance.
        .maxWaitTime = static_cast<uint32_t>(
                       1000.0f * (M_TWOPI / 8) / 0.2f * 20 *
                       (M_TWOPI / 8)) +
                   100,
        .targetCycleCount = 3,
    },
    unjamAgitatorCommand(
        agitatorSubsystem,
        unjamAgitatorCommandConfig),
    rotateAndUnjamAgitatorCommand(
        drivers, 
        agitatorSubsystem,
        rotateAgitatorCommand,
        unjamAgitatorCommand),
    leftMousePressed(
        &drivers,
        {&rotateAndUnjamAgitatorCommand},
        RemoteMapState(
            RemoteMapState::MouseButton::LEFT)),
    rightMousePressed(
        &drivers,
        {&rotateAndUnjamAgitatorCommand},
        RemoteMapState(
            RemoteMapState::MouseButton::RIGHT),
            true),
                m_FlyWheel(
        drivers,
        &drivers.pwm,
        tap::gpio::Pwm::C6,
        tap::gpio::Pwm::C7),
    m_ControlOperatorInterface(drivers.remote),
    m_FlyWheelCommand(m_FlyWheel, m_ControlOperatorInterface),
        leftSwitchUp(
        &drivers,
        {&rotateAndUnjamAgitatorCommand},
        RemoteMapState(
            Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP), 
            true ),
    pitchMotor(
        &drivers, 
        MotorId::MOTOR5, 
        CanBus::CAN_BUS1, 
        true, "pitchMotor"),
    turretPitchMotor(
        &pitchMotor,
        PITCH_MOTOR_CONFIG),
    yawMotor(
        &drivers, 
        MotorId::MOTOR8, 
        CanBus::CAN_BUS1, 
        false, "YawMotor"),
    turretGyro(
        &drivers),
    turretYawMotor(
        &yawMotor,
        YAW_MOTOR_CONFIG,
        &turretGyro
    ),
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
        m_ControlOperatorInterface,
        &turret, 
        &yawController,
        &pitchController,
        USER_YAW_INPUT_SCALAR,
        USER_PITCH_INPUT_SCALAR,
        0
    ),
        turretOrientedDriveCommand(
        &drivers,
        m_ControlOperatorInterface,
        &m_ChassisSubsystem,
        &turretYawMotor
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
    m_ChassisSubsystem.initialize();
    agitatorSubsystem.initialize();
    m_FlyWheel.initialize();
    turret.initialize();
}

void Robot::registerSoldierSubsystems()
{
    drivers.commandScheduler.registerSubsystem(&m_ChassisSubsystem);
    drivers.commandScheduler.registerSubsystem(&agitatorSubsystem);
    drivers.commandScheduler.registerSubsystem(&m_FlyWheel);
    drivers.commandScheduler.registerSubsystem(&turret);
}

void Robot::setDefaultSoldierCommands()
{
    m_ChassisSubsystem.setDefaultCommand(&turretOrientedDriveCommand);
    m_FlyWheel.setDefaultCommand(&m_FlyWheelCommand);
    turret.setDefaultCommand(&turretUserControlCommand);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{
    drivers.commandMapper.addMap(&leftMousePressed);
    drivers.commandMapper.addMap(&rightMousePressed);
    drivers.commandMapper.addMap(&leftSwitchUp);

}   
}  // namespace control
