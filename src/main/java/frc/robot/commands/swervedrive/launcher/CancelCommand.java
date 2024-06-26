package frc.robot.commands.swervedrive.launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Command for note intake
 */
public class CancelCommand extends Command 
{

  private final LauncherSubsystem launcher;
  private final DoubleSupplier doubleSupplier;  

  /**
   * Cancel all launcher mechanisms
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public CancelCommand(LauncherSubsystem launcher, DoubleSupplier doubleSupplier)
  {

    this.launcher = launcher;
    this.doubleSupplier = doubleSupplier;

    addRequirements(launcher);
  }

  /**
   * Cancel all launcher mechanisms
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public CancelCommand(LauncherSubsystem launcher)
  {

    this.launcher = launcher;
    this.doubleSupplier = () -> 0;
    addRequirements(launcher);
  }

  @Override
  public void initialize()
  {
    // Set all motors to stop
    this.launcher.setIntakeSpeed(0);
    this.launcher.setLoaderSpeed(0);
    this.launcher.setAssistSpeed(0);
    this.launcher.setLaunchSpeed(0);
    this.launcher.setElevatorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    System.out.print("Distance measure: ");
    System.out.println(doubleSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    this.launcher.setIntakeSpeed(0);
    this.launcher.setLoaderSpeed(0);
    this.launcher.setAssistSpeed(0);
    this.launcher.setLaunchSpeed(0);
    this.launcher.setElevatorSpeed(0);
  }

  // Returns
  @Override
  public boolean isFinished()
  {
    return true;
  }
}


  
