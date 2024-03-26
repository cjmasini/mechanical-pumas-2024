package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.launcher.LauncherSubsystem;

/**
 * Command for note intake
 */
public class SpeakerCommand extends SequentialCommandGroup 
{

  private final LauncherSubsystem launcher;
  
  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public SpeakerCommand(LauncherSubsystem launcher)
  {
    this.launcher = launcher;

    Command setLaunchWheelSpeed = new InstantCommand(() -> this.launcher.setLaunchSpeed(1), launcher);
    WaitCommand waitCommand = new WaitCommand(1);
    Command setLoadWheelSpeed = new InstantCommand(() -> this.launcher.setLoaderSpeed(1), launcher);
    this.addCommands(setLaunchWheelSpeed, waitCommand, setLoadWheelSpeed);
    addRequirements(launcher);
  }
}


  
