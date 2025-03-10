/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_GOVERNOR_WITH_FALLBACK_COMMAND_HPP_
#define TAPROOT_GOVERNOR_WITH_FALLBACK_COMMAND_HPP_

#include <algorithm>
#include <array>
#include <cassert>
#include <cinttypes>
#include <vector>

// #include "../command.hpp"

#include "command_governor_interface.hpp"

namespace tap::control::governor
{
/**
 * A command that alternates between two commands based on the state of a list of governors. One of
 * the two commands is selected in the `isReady` phase of the Command, at which point that command
 * will be executed until its completion.
 *
 * @tparam NUM_CONDITIONS The number of governors in the governor list.
 */
template <std::size_t NUM_CONDITIONS>
class GovernorWithFallbackCommand : public Command
{
public:
    GovernorWithFallbackCommand(
        std::vector<Subsystem *> subRequirements,
        Command &commandWhenGovernorsReady,
        Command &fallbackCommand,
        const std::array<CommandGovernorInterface *, NUM_CONDITIONS> &commandGovernorList)
        : commandWhenGovernorsReady(commandWhenGovernorsReady),
          fallbackCommand(fallbackCommand),
          commandGovernorList(commandGovernorList)
    {
        std::for_each(subRequirements.begin(), subRequirements.end(), [&](auto sub) {
            addSubsystemRequirement(sub);
        });

        assert(
            commandWhenGovernorsReady.getRequirementsBitwise() == this->getRequirementsBitwise());
        assert(fallbackCommand.getRequirementsBitwise() == this->getRequirementsBitwise());
    }

    const char *getName() const override { return "Governor w/fallback"; }

    bool isReady() override
    {
        currentGovernorReadiness =
            std::all_of(commandGovernorList.begin(), commandGovernorList.end(), [](auto governor) {
                return governor->isReady();
            });

        return (currentGovernorReadiness && commandWhenGovernorsReady.isReady()) ||
               (!currentGovernorReadiness && fallbackCommand.isReady());
    }

    void initialize() override
    {
        if (currentGovernorReadiness)
        {
            commandWhenGovernorsReady.initialize();
        }
        else
        {
            fallbackCommand.initialize();
        }
    }

    void execute() override
    {
        if (currentGovernorReadiness)
        {
            commandWhenGovernorsReady.execute();
        }
        else
        {
            fallbackCommand.execute();
        }
    }

    void end(bool interrupted) override
    {
        if (currentGovernorReadiness)
        {
            commandWhenGovernorsReady.end(interrupted);
        }
        else
        {
            fallbackCommand.end(interrupted);
        }
    }

    bool isFinished() const override
    {
        return currentGovernorReadiness ? commandWhenGovernorsReady.isFinished()
                                        : fallbackCommand.isFinished();
    }

private:
    bool currentGovernorReadiness = false;
    Command &commandWhenGovernorsReady;
    Command &fallbackCommand;

    std::array<CommandGovernorInterface *, NUM_CONDITIONS> commandGovernorList;
};
}  // namespace tap::control::governor

#endif  // TAPROOT_GOVERNOR_WITH_FALLBACK_COMMAND_HPP_
