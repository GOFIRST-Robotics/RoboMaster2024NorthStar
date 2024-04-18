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

#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"


using tap::algorithms::limitVal;

namespace control::chassis
{
// STEP 1 (Tank Drive): create constructor
ChassisSubsystem::ChassisSubsystem(src::Drivers& drivers, const ChassisConfig& config) : Subsystem(&drivers),
desiredOutput{}, 
//controllers are designated in this order, front left, back left, front right, back right
pidControllers{},
motors{{
        Motor(&drivers, config.leftFrontId, config.canBus, false, "Front Left Drive Motor"),
        Motor(&drivers, config.leftBackId, config.canBus, false, "Back Left Drive Motor"),
        Motor(&drivers, config.rightFrontId, config.canBus, true, "Front Right Drive Motor"),
        Motor(&drivers, config.rightBackId, config.canBus, true, "Back Right Drive Motor")
}}
{
    for(int i = 0; i < 4; i++) {
        pidControllers[i].setParameter(config.wheelVelocityPidConfig);
    }
}

// STEP 2 (Tank Drive): initialize function
void ChassisSubsystem::initialize() {
    for (int i = 0; i < 4; i++) {
        motors[i].initialize();
    }
}

// STEP 3 (Tank Drive): setVelocityTankDrive function, input should be in chassis speed in m/s
void  ChassisSubsystem::setVelocityMecanumDrive(float translationHorizontal, float translationVertical, float rotation) {

    float left_front_input = translationHorizontal + translationVertical + rotation;
    float left_back_input = -translationHorizontal + translationVertical + rotation;
    float right_front_input = -translationHorizontal + translationVertical - rotation;
    float right_back_input = translationHorizontal + translationVertical - rotation;

    float maximum_input = std::max<float>({std::abs(left_front_input), std::abs(left_back_input), std::abs(right_front_input), std::abs(right_back_input)});
    if(maximum_input != 0.0f) {
        if(maximum_input > MAX_CHASSIS_SPEED_MPS) {
            left_front_input = left_front_input / maximum_input * MAX_CHASSIS_SPEED_MPS;
            left_back_input = left_back_input / maximum_input * MAX_CHASSIS_SPEED_MPS;
            right_front_input = right_front_input / maximum_input * MAX_CHASSIS_SPEED_MPS;
            right_back_input = right_back_input / maximum_input * MAX_CHASSIS_SPEED_MPS;
        }
    }

    float left_front_rpm = mpsToRpm(left_front_input);
    float left_back_rpm = mpsToRpm(left_back_input);
    float right_front_rpm = mpsToRpm(right_front_input);
    float right_back_rpm = mpsToRpm(right_back_input);

    float left_front_drive_output = limitVal<float>(left_front_rpm, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    float left_back_drive_output = limitVal<float>(left_back_rpm, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    float right_front_drive_output = limitVal<float>(right_front_rpm, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    float right_back_drive_output = limitVal<float>(right_back_rpm, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);

    desiredOutput[static_cast<int>(MotorId::LF)] = left_front_drive_output;
    desiredOutput[static_cast<int>(MotorId::LB)] = left_back_drive_output;
    desiredOutput[static_cast<int>(MotorId::RF)] = right_front_drive_output;
    desiredOutput[static_cast<int>(MotorId::RB)] = right_back_drive_output;
}

// STEP 4 (Tank Drive): refresh function
void ChassisSubsystem::refresh() {
    auto runPid = [](Pid &pid, Motor &motor, float desiredOutput) {
        pid.update(desiredOutput - motor.getShaftRPM());
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
    }
}
}  // namespace control::chassis
