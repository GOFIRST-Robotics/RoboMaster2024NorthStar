/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "additionalTapResources/control_operator_additional.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"


using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace control
{



float ControlOperatorInterface::getTurretYawInput(uint8_t turretID)
{
    switch (turretID)
    {
        case 0:
            return -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) +
                   static_cast<float>(limitVal<int16_t>(
                       -drivers->remote.getMouseX(),
                       -USER_MOUSE_YAW_MAX,
                       USER_MOUSE_YAW_MAX)) *
                       USER_MOUSE_YAW_SCALAR;
        case 1:
            return -drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) +
                   static_cast<float>(limitVal<int16_t>(
                       -drivers->remote.getMouseX(),
                       -USER_MOUSE_YAW_MAX,
                       USER_MOUSE_YAW_MAX)) *
                       USER_MOUSE_YAW_SCALAR;
        default:
            return 0;
    }
}


float ControlOperatorInterface::getTurretPitchInput(uint8_t turretID)
{
    switch (turretID)
    {
        case 0:
            return -drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) +
                   static_cast<float>(limitVal<int16_t>(
                       drivers->remote.getMouseY(),
                       -USER_MOUSE_PITCH_MAX,
                       USER_MOUSE_PITCH_MAX)) *
                       USER_MOUSE_PITCH_SCALAR;
        case 1:
            return -drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) +
                   static_cast<float>(limitVal<int16_t>(
                       drivers->remote.getMouseY(),
                       -USER_MOUSE_PITCH_MAX,
                       USER_MOUSE_PITCH_MAX)) *
                       USER_MOUSE_PITCH_SCALAR;
        default:
            return 0;
    }
}


}  // namespace control
