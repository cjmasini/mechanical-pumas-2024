package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Command for note intake
 */
public class IntakeCommand extends Command 
{

  private static final int ULTRASONIC_SENSOR_DISTANCE_MM = 150;
  private final LauncherSubsystem launcher;
  private final DoubleSupplier ultrasonicDistance;

  private boolean intakeFinished = true;
  
  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public IntakeCommand(LauncherSubsystem launcher, DoubleSupplier ultrasonicDistance)
  {
    this.launcher = launcher;
    this.ultrasonicDistance = ultrasonicDistance;

    addRequirements(launcher);
  }

  @Override
  public void initialize()
  {
    System.out.print("Initializing");
    this.intakeFinished = false;
    this.launcher.setIntakeSpeed(0.5);
    this.launcher.setLoaderSpeed(0.5);
    this.launcher.setAssistSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    //If ultrasonic sensor is not blocked when left bumper is pressed, then load note from intake and position wheel until
    //sensor is blocked
    double distance = this.ultrasonicDistance.getAsDouble();
    // System.out.print(distance);
    if(distance <= ULTRASONIC_SENSOR_DISTANCE_MM && distance > 15){
      this.intakeFinished = true;
    }  
  }  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    this.intakeFinished = true;
    this.launcher.setIntakeSpeed(0);
    this.launcher.setLoaderSpeed(0);
    this.launcher.setAssistSpeed(0);
  }

  // Returns
  @Override
  public boolean isFinished()
  {
    return this.intakeFinished;
  }
}


  
