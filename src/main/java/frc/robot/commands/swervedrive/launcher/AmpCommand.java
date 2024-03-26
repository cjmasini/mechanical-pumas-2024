package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.LauncherSubsystem;

/**
 * Command for note intake
 */
public class AmpCommand extends Command 
{

  private final LauncherSubsystem launcher;
  private final double launchSpeed;

  private boolean ampLaunchFinished = true;
  
  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public AmpCommand(LauncherSubsystem launcher, double launchSpeed)
  {
    this.launcher = launcher;
    this.launchSpeed = launchSpeed;

    addRequirements(launcher);
  }

  @Override
  public void initialize()
  {
    this.ampLaunchFinished = false;
    // Run load and launch motors at low speed
    this.launcher.setLoaderSpeed(this.launchSpeed);
    this.launcher.setLaunchSpeed(this.launchSpeed);
    // this.withTimeout(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
      // TODO: Figure out timeout
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    this.ampLaunchFinished = true;
    this.launcher.setLoaderSpeed(0);
    this.launcher.setLaunchSpeed(0);
  }

  // Returns
  @Override
  public boolean isFinished()
  {
    return this.ampLaunchFinished;
  }
}


  
