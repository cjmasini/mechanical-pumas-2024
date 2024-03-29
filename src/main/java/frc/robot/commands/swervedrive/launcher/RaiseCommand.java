package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Command for note intake
 */
public class RaiseCommand extends Command 
{
  private final LauncherSubsystem launcher;

  private boolean raiseFinished = true;
  
  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   */
  public RaiseCommand
(LauncherSubsystem launcher)
  {
    this.launcher = launcher;

    addRequirements(launcher);
  }

  @Override
  public void initialize()
  {
    this.raiseFinished = false;
    // Run elevator motors at max speed
    this.launcher.setElevatorSpeed(1);
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
    this.raiseFinished = true;
    this.launcher.setElevatorSpeed(0);
  }

  // Returns
  @Override
  public boolean isFinished()
  {
    return this.raiseFinished;
  }
}


  
