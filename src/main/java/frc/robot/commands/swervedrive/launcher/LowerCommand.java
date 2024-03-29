package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Command for note intake
 */
public class LowerCommand extends Command 
{
  private final LauncherSubsystem launcher;

  private boolean loweringFinished = true;
  
  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   */
  public LowerCommand
(LauncherSubsystem launcher)
  {
    this.launcher = launcher;

    addRequirements(launcher);
  }

  @Override
  public void initialize()
  {
    this.loweringFinished = false;
    // Run elevator motors at max speed
    this.launcher.setElevatorSpeed(-1);
    // this.withTimeout(2);
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
    this.loweringFinished = true;
    this.launcher.setElevatorSpeed(0);
  }

  // Returns
  @Override
  public boolean isFinished()
  {
    return this.loweringFinished;
  }
}


  
