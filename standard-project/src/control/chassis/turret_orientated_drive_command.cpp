/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "turret_orientated_drive_command.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"


using namespace tap::communication::sensors::imu::bmi088;

namespace control::chassis
{
ChassisTurretDriveCommand::ChassisTurretDriveCommand(
    tap::Drivers* drivers,
    control::ControlOperatorInterface& operatorInterface,
    control::chassis::ChassisSubsystem* chassis,
    const control::turret::YawTurretMotor* yawMotor)
    : tap::control::Command(),
      drivers(drivers),
      operatorInterface(operatorInterface),
      chassis(chassis),
      yawMotor(yawMotor),
      rotationSetpoint(0, 0, M_TWOPI)
{
    addSubsystemRequirement(chassis);
}

void ChassisTurretDriveCommand::execute()
{
    float chassisRotationDesiredWheelspeed = 0.0f;
    chassisRotationDesiredWheelspeed = operatorInterface.getMecanumRotationKeyBoard();

    float chassisXDesiredWheelspeed = 0.0f;
    float chassisYDesiredWheelspeed = 0.0f;

    chassis->computeDesiredUserTranslation(
        &operatorInterface,
        drivers,
        chassis,
        chassisRotationDesiredWheelspeed,
        &chassisXDesiredWheelspeed,
        &chassisYDesiredWheelspeed);

    if (yawMotor != nullptr && yawMotor->isOnline())
    {
        // rotate X and Y based on turret angle from center so translational motion is relative
        // to the turret
        tap::algorithms::rotateVector(
            &chassisXDesiredWheelspeed,
            &chassisYDesiredWheelspeed,
            -yawMotor->getChassisDriveOffset());
    }

    chassis->setVelocityMecanumDriveWithWheels(
        chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed,
        chassisRotationDesiredWheelspeed);
}

void ChassisTurretDriveCommand::end(bool) { chassis->setVelocityMecanumDriveWithWheels(0, 0, 0);}

bool ChassisTurretDriveCommand::isFinished() const { return false; }
}  // namespace aruwsrc::chassis
