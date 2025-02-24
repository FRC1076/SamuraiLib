// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.extendedcommands;

import java.util.function.BooleanSupplier;

import org.apache.commons.lang3.NotImplementedException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CommandUtils {
    private CommandUtils() {
        throw new NotImplementedException("This is a utility class!");
    }

    public static Command makeDaemon(Command command, BooleanSupplier endCondition) {
        return new DaemonCommand(command, endCondition);
    }

    /** tells the robot to periodically run a runnable that is not associated with any particular subsystem */
    public static void makePeriodic(Runnable runnable, boolean runWhenDisabled) {
        CommandScheduler.getInstance().schedule(Commands.run(runnable).ignoringDisable(runWhenDisabled));
    }

    /** tells the robot to periodically run a runnable that is not associated with any particular subsystem */
    public static void makePeriodic(Runnable runnable) {
        CommandScheduler.getInstance().schedule(Commands.run(runnable).ignoringDisable(false));
    }
}
