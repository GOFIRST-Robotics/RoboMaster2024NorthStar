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

#include "control/flywheel/fly_wheel_subsystem.hpp"
#include "control/flywheel/fly_wheel_shoot_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/control_operator_interface.hpp"


using tap::algorithms::limitVal;

namespace control::flyWheel
{
// STEP 1 (Tank Drive): Constructor
flyWheelCommand::flyWheelCommand(
    control::flyWheel::FlyWheelSubsystem &flyWheel, control::ControlOperatorInterface &operatorInterface) :
    flyWheel(flyWheel),
    operatorInterface(operatorInterface)
{
    addSubsystemRequirement(&flyWheel);
}

void flyWheelCommand::initialize(){
    flyWheel.initialize();
}



// STEP 2 (Tank Drive): execute function
void flyWheelCommand::execute()
{
    if(operatorInterface.isRightSwitchUp()){
        flyWheel.setMaxOutput();
    } else {
        flyWheel.disable();
    }
}

void flyWheelCommand::end(bool interrupted){
    flyWheel.disable();
}


};  // namespace control
