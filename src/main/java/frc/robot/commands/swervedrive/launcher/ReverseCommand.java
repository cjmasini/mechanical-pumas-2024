package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.LauncherSubsystem;

/**
 * Command for note intake
 */
public class ReverseCommand extends Command 
{

  private final LauncherSubsystem launcher;

  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public ReverseCommand(LauncherSubsystem launcher)
  {

    this.launcher = launcher;

    addRequirements(launcher);
  }

  @Override
  public void initialize()
  {
    // Run load and launch motors at low speed
    this.launcher.setIntakeSpeed(-.25);
    this.launcher.setLoaderSpeed(-.25);
    this.launcher.setLaunchSpeed(-.75);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    this.launcher.setIntakeSpeed(0);
    this.launcher.setLoaderSpeed(0);
    this.launcher.setLaunchSpeed(0);
  }

  // Returns
  @Override
  public boolean isFinished()
  {
    return false;
  }
}


  
