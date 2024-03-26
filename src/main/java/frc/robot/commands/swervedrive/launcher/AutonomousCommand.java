package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;

/**
 * Command for note intake
 */
public class AutonomousCommand extends Command 
{

  private final DriveSubsystem driveSubsystem;
  private double time;


  private boolean ampLaunchFinished = true;
  
  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public AutonomousCommand(DriveSubsystem driveSubsystem)
  {
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize()
  {
    this.driveSubsystem.drive(.03, 0, 0, true, true);
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
    this.driveSubsystem.drive(0, 0, 0, true, true);
  }

  // Returns
  @Override
  public boolean isFinished()
  {
    return true;
  }
}


  
