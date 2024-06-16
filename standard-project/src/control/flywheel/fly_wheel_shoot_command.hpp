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

#include "tap/control/command.hpp"
#include "control/flywheel/fly_wheel_subsystem.hpp"
#include "control/control_operator_interface.hpp"

namespace control::flyWheel
{
    class flyWheelCommand : public tap::control::Command
    {
    public:
        flyWheelCommand(flyWheel::FlyWheelSubsystem &flyWheel, ControlOperatorInterface &operatorInterface);

        const char *getName() const override { return "Chassis tank drive"; }

        void initialize() override;

        void execute() override;

        void end(bool interrupted) override;

        bool isFinished() const { return false;}

    private:
        control::flyWheel::FlyWheelSubsystem &flyWheel;

        ControlOperatorInterface &operatorInterface;

        float calibrationTimeStart = 0;

        
    };
}  // namespace control::chassis