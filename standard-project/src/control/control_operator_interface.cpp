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

#include "control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"
#include <tap/architecture/clock.hpp>

using tap::algorithms::limitVal;
using tap::communication::serial::Remote;

namespace control
{
ControlOperatorInterface::ControlOperatorInterface(Remote &remote) : remote(remote) {}

template <typename T>
int getSign(T val)
{
    return (T(0) < val) - (val < T(0));
}



float ControlOperatorInterface::applyChassisSpeedScaling(float value)
{
    if (isSlowMode())
    {
        value *=  0.333; //SPEED_REDUCTION_SCALAR;
    }
    return value;
}

bool ControlOperatorInterface::isSlowMode()
{
    return remote.keyPressed(Remote::Key::CTRL);
}

/**
 * @param[out] ramp Ramp that should have acceleration applied to. The ramp is updated some
 * increment based on the passed in acceleration values. Ramp stores values in some units.
 * @param[in] maxAcceleration Positive acceleration value to apply to the ramp in units/time^2.
 * @param[in] maxDeceleration Negative acceleration value to apply to the ramp, in units/time^2.
 * @param[in] dt Change in time since this function was last called, in units of some time.
 */
static inline void applyAccelerationToRamp(
    tap::algorithms::Ramp &ramp,
    float maxAcceleration,
    float maxDeceleration,
    float dt)
{
    if (getSign(ramp.getTarget()) == getSign(ramp.getValue()) &&
        abs(ramp.getTarget()) > abs(ramp.getValue()))
    {
        // we are trying to speed up
        ramp.update(maxAcceleration * dt);
    }
    else
    {
        // we are trying to slow down
        ramp.update(maxDeceleration * dt);
    }
}
// STEP 2 (Tank Drive): Add getChassisTankLeftInput and getChassisTankRightInput function
// definitions

float ControlOperatorInterface::getMecanumHorizontalTranslation() {
    if(remote.keyPressed(Remote::Key::A) && !remote.keyPressed(Remote::Key::SHIFT)){
        return -0.2f;
    } else if (remote.keyPressed(Remote::Key::A) && remote.keyPressed(Remote::Key::SHIFT)){
        return -0.4f;
    } else if (remote.keyPressed(Remote::Key::D) && !remote.keyPressed(Remote::Key::SHIFT)){
        return 0.2f;
    } else if (remote.keyPressed(Remote::Key::D) && remote.keyPressed(Remote::Key::SHIFT)){
        return 0.4f;
    } else {
        return 0.0f;
    }
}

float keyInputDebug = 0.0f;
float finalXDebug = 0.0f;
float outputDebug = 0.0f;
float ControlOperatorInterface::getMecanumHorizontalTranslationKeyBoard() {

    uint32_t updateCounter = remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisXInputCalledTime;
    prevChassisXInputCalledTime = currTime;

    if (prevUpdateCounterX != updateCounter)
    {
        chassisXInput.update(remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterX = updateCounter;
    }

    float keyInput =
        remote.keyPressed(Remote::Key::W) - remote.keyPressed(Remote::Key::S);
    keyInputDebug = keyInput;

    const float maxChassisSpeed = 7000;

    float finalX = maxChassisSpeed *
                   limitVal(chassisXInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);
    finalXDebug = finalX;

    chassisXInputRamp.setTarget(applyChassisSpeedScaling(finalX));

    applyAccelerationToRamp(
        chassisXInputRamp,
        MAX_ACCELERATION_X,
        MAX_DECELERATION_X,
        static_cast<float>(dt) / 1E3F);
    outputDebug = chassisXInputRamp.getValue();
    return outputDebug;
}

float ControlOperatorInterface::getMecanumVerticalTranslation() {
    if(remote.keyPressed(Remote::Key::W) && !remote.keyPressed(Remote::Key::SHIFT)){
        return 0.2f;
    } else if (remote.keyPressed(Remote::Key::W) && remote.keyPressed(Remote::Key::SHIFT)){
        return 0.4f;
    } else if (remote.keyPressed(Remote::Key::S) && !remote.keyPressed(Remote::Key::SHIFT)){
        return -0.2f;
    } else if (remote.keyPressed(Remote::Key::S) && remote.keyPressed(Remote::Key::SHIFT)){
        return -0.4f;
    } else {
        return 0.0f;
    }
}


float ControlOperatorInterface::getMecanumVerticalTranslationKeyBoard() {
uint32_t updateCounter = remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisYInputCalledTime;
    prevChassisYInputCalledTime = currTime;

    if (prevUpdateCounterY != updateCounter)
    {
        chassisYInput.update(
            -remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),
            currTime);
        prevUpdateCounterY = updateCounter;
    }

    float keyInput =
        remote.keyPressed(Remote::Key::A) - remote.keyPressed(Remote::Key::D);

    const float maxChassisSpeed = 7000;

    float finalY = maxChassisSpeed *
                   limitVal(chassisYInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisYInputRamp.setTarget(applyChassisSpeedScaling(finalY));

    applyAccelerationToRamp(
        chassisYInputRamp,
        MAX_ACCELERATION_Y,
        MAX_DECELERATION_Y,
        static_cast<float>(dt) / 1E3F);

    return chassisYInputRamp.getValue();
}

float ControlOperatorInterface::getMecanumRotation()
{
    if(remote.keyPressed(Remote::Key::CTRL)){
        return 0.2f;
    } else {
        return 0;
    }
}


float ControlOperatorInterface::getMecanumRotationKeyBoard()
{
    uint32_t updateCounter = remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisRInputCalledTime;
    prevChassisRInputCalledTime = currTime;

    if (prevUpdateCounterR != updateCounter)
    {
        chassisRInput.update(
            -remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
            currTime);
        prevUpdateCounterR = updateCounter;
    }

    float keyInput =
        remote.keyPressed(Remote::Key::Q) - remote.keyPressed(Remote::Key::E);

    const float maxChassisSpeed = 7000;

    float finalR = maxChassisSpeed *
                   limitVal(chassisRInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisRInputRamp.setTarget(finalR);

    applyAccelerationToRamp(
        chassisRInputRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return chassisRInputRamp.getValue();
}



    bool ControlOperatorInterface::isRightSwitchUp(){
        return (remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP);
    }

    bool ControlOperatorInterface::isGKeyPressed(){
        return (remote.keyPressed(Remote::Key::G));
    }
    
}  // namespace control
