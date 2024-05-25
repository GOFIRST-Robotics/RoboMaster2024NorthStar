/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "turret_gyro.hpp"
#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"



namespace control::turret 
{
    TurretMCBCGryo::TurretMCBCGryo(src::Drivers* drivers) : drivers(drivers) {}

    float TurretMCBCGryo::getRoll() { 
    prevRoll = currRoll;
    currRoll = drivers->bmi088.getRoll(); 
    updateRevolutionCounter(currRoll, prevRoll, rollRevolutions);
    return currRoll;
    }

    float TurretMCBCGryo::getRollUnwrapped() const { 
        return currRoll + M_TWOPI * static_cast<float>(rollRevolutions);
    }

    float TurretMCBCGryo::getPitch() { 
        prevPitch = currPitch;
        currPitch = drivers->bmi088.getPitch(); 
        updateRevolutionCounter(currPitch, prevPitch, pitchRevolutions);
        return currPitch;
    }

    float TurretMCBCGryo::getPitchUnwrapped() const { 
        return currPitch + M_TWOPI * static_cast<float>(pitchRevolutions);
    }

    float TurretMCBCGryo::getYaw() { 
        prevYaw = currYaw;
        currYaw = drivers->bmi088.getYaw(); 
        updateRevolutionCounter(currYaw, prevYaw, yawRevolutions);
        return currYaw;
    }

    float TurretMCBCGryo::getYawUnwrapped() const { 
        return currYaw + M_TWOPI * static_cast<float>(yawRevolutions);
    }
}  // namespace aruwsrc::can