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

#include "tap/util_macros.hpp"
#include <tap/algorithms/linear_interpolation_predictor.hpp>
#include <tap/algorithms/ramp.hpp>

namespace tap::communication::serial
{
class Remote;
}

namespace control
{
class ControlOperatorInterface
{
public:
    ControlOperatorInterface(tap::communication::serial::Remote &remote);


    static constexpr float MAX_ACCELERATION_X = 10'000.0f;
    static constexpr float MAX_DECELERATION_X = 20'000.0f;

    static constexpr float MAX_ACCELERATION_Y = 9'000.0f;
    static constexpr float MAX_DECELERATION_Y = 20'000.0f;

    static constexpr float MAX_ACCELERATION_R = 40'000.0f;
    static constexpr float MAX_DECELERATION_R = 50'000.0f;

    // STEP 1 (Tank Drive): Add getChassisTankLeftInput and getChassisTankRightInput function
    // declarations
    float getMecanumHorizontalTranslation();

    float getMecanumHorizontalTranslationKeyBoard();

    float getMecanumVerticalTranslation();

    float getMecanumVerticalTranslationKeyBoard();

    float getMecanumRotation();

    float getMecanumRotationKeyBoard();

    /**
     * @returns whether or not the key to disable diagonal drive is pressed.
     * The key is shared with the speed scaling key.
     */
    bool isSlowMode();

private:
    tap::communication::serial::Remote &remote;

    uint32_t prevUpdateCounterX = 0;
    uint32_t prevUpdateCounterY = 0;
    uint32_t prevUpdateCounterR = 0;

    tap::algorithms::LinearInterpolationPredictor chassisXInput;
    tap::algorithms::LinearInterpolationPredictor chassisYInput;
    tap::algorithms::LinearInterpolationPredictor chassisRInput;

    tap::algorithms::Ramp chassisXInputRamp;
    tap::algorithms::Ramp chassisYInputRamp;
    tap::algorithms::Ramp chassisRInputRamp;

    uint32_t prevChassisXInputCalledTime = 0;
    uint32_t prevChassisYInputCalledTime = 0;
    uint32_t prevChassisRInputCalledTime = 0;

    /**
     * Scales `value` when ctrl/shift are pressed and returns the scaled value.
     */
    float applyChassisSpeedScaling(float value);
};
}  // namespace control
