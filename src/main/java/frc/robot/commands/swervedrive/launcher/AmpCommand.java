package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Command for note intake
 */
public class AmpCommand extends SequentialCommandGroup 
{

  private final LauncherSubsystem launcher;
  
  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public AmpCommand(LauncherSubsystem launcher)
  {
    this.launcher = launcher;

    Command setLaunchWheelSpeed = new InstantCommand(() -> this.launcher.setLaunchSpeed(.33), launcher);
    InstantCommand setAssistWheelSpeed = new InstantCommand(() -> this.launcher.setAssistSpeed(.5));
    Command setLoadWheelSpeed = new InstantCommand(() -> this.launcher.setLoaderSpeed(.5), launcher);
    this.addCommands(setLaunchWheelSpeed, setAssistWheelSpeed, setLoadWheelSpeed);
    addRequirements(launcher);
  }
}
