package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Takes instant command with no end condition and runs cancel command after supplied timeout
 * @param command - instant command to run for time
 * @param seconds - time to run command for
 */
public class TimeoutCommand extends SequentialCommandGroup {
    
    public TimeoutCommand(Command startCommand, double seconds, Command endCommand) {
        WaitCommand waitCommand = new WaitCommand(seconds);
        this.addCommands(startCommand, waitCommand, endCommand);
    }
}
