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

#include "control/control_operator_interface.hpp"


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


void  ChassisSubsystem::setVelocityMecanumDriveWithWheels(float x, float y, float r) {

    // this is the distance between the center of the chassis to the wheel
    float maxWheelSpeed = MAX_WHEELSPEED_RPM;
    float chassisRotationRatio = sqrtf(
        powf(0.366 / 2.0f, 2.0f) + powf(0.385 / 2.0f, 2.0f));

    // to take into account the location of the turret so we rotate around the turret rather
    // than the center of the chassis, we calculate the offset and than multiply however
    // much we want to rotate by
    float leftFrontRotationRatio =
        modm::toRadian(chassisRotationRatio);
    float rightFrontRotationRatio =
        modm::toRadian(chassisRotationRatio);
    float leftBackRotationRatio =
        modm::toRadian(chassisRotationRatio);
    float rightBackRotationRatio =
        modm::toRadian(chassisRotationRatio);

    float chassisRotateTranslated = modm::toDegree(r) / chassisRotationRatio;
    desiredOutput[static_cast<int>(MotorId::LF)] = limitVal(
        -y + x - chassisRotateTranslated * leftFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredOutput[static_cast<int>(MotorId::RF)] = limitVal(
        -(-y - x - chassisRotateTranslated * rightFrontRotationRatio),
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredOutput[static_cast<int>(MotorId::LB)] = limitVal(
        y + x - chassisRotateTranslated * leftBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredOutput[static_cast<int>(MotorId::RB)] = limitVal(
        -(y - x - chassisRotateTranslated * rightBackRotationRatio),
        -maxWheelSpeed,
        maxWheelSpeed);

    // desiredRotation = r;
}



void ChassisSubsystem::computeDesiredUserTranslation(
    control::ControlOperatorInterface *operatorInterface,
    tap::Drivers *drivers,
    ChassisSubsystem *chassis,
    float chassisRotation,
    float *chassisXDesiredWheelspeed,
    float *chassisYDesiredWheelspeed)
{
    if (drivers == nullptr || operatorInterface == nullptr || chassis == nullptr ||
        chassisXDesiredWheelspeed == nullptr || chassisYDesiredWheelspeed == nullptr)
    {
        return;
    }


    const float maxWheelSpeed = 7000;
    // const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
    //     drivers->refSerial.getRefSerialReceivingData(),
    //     drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

    // what we will multiply x and y speed by to take into account rotation
    float rotationLimitedMaxTranslationalSpeed =
        calculateRotationTranslationalGain(chassisRotation) * maxWheelSpeed;

    *chassisXDesiredWheelspeed = limitVal(
        operatorInterface->getMecanumHorizontalTranslationKeyBoard(),
        -rotationLimitedMaxTranslationalSpeed,
        rotationLimitedMaxTranslationalSpeed);

    *chassisYDesiredWheelspeed = limitVal(
        operatorInterface->getMecanumVerticalTranslationKeyBoard(),
        -rotationLimitedMaxTranslationalSpeed,
        rotationLimitedMaxTranslationalSpeed);
}



float ChassisSubsystem::calculateRotationTranslationalGain(
    float chassisRotationDesiredWheelspeed)
{
    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the wheel rotation speed for chassis rotation is greater than the
    // MIN_ROTATION_THRESHOLD
    if (fabsf(chassisRotationDesiredWheelspeed) > 800.0) //Min_rotation_threshold
    {
        const float maxWheelSpeed = 7000;

        // power(max revolve speed + min rotation threshold - specified revolve speed, 2) /
        // power(max revolve speed, 2)
        rTranslationalGain = powf(
            (maxWheelSpeed + 800.0 - fabsf(chassisRotationDesiredWheelspeed)) /
                maxWheelSpeed,
            2.0f);

        rTranslationalGain = limitVal(rTranslationalGain, 0.0f, 1.0f);
    }
    return rTranslationalGain;
}

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
