package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command for note intake
 */
public class LockOrientationCommand extends Command 
{
  public static final double ANGLE_OFFSET = 2; // degrees 
  private CommandXboxController driverXbox;
  private DriveSubsystem driveSubsystem;
  private Direction direction;
  private boolean matchesDirection = false;

  public enum Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
  }

  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public LockOrientationCommand(DriveSubsystem driveSubsystem, CommandXboxController driverXbox, Direction direction)
  {
      this.driveSubsystem = driveSubsystem;
      this.driverXbox = driverXbox;
      this.direction = direction;
      addRequirements(this.driveSubsystem);
  }

  @Override
  public void execute() {

    System.out.println(this.convertToSmallAngle(this.driveSubsystem.getHeading()));

    double angle = convertToSmallAngle(this.driveSubsystem.getHeading());

    if (this.matchesDirectionWithinTolerance(angle, this.direction)){
      this.matchesDirection = true;
    }
    double yMovement = driverXbox.getLeftY();
    double xMovement = driverXbox.getLeftX();
    // The left stick controls translation of the robot.
    // Automatically turn to face the supplied direction
    
    System.out.println("Matches within tolerance: " + this.matchesDirectionWithinTolerance(angle, Direction.FORWARD)); 
    System.out.println("Speed: "+ (this.matchesDirectionWithinTolerance(angle, Direction.FORWARD) ? 0 : calculateAngularSpeed(angle, direction)));
    this.driveSubsystem.drive(
        -MathUtil.applyDeadband((yMovement > 0 ? 1 : -1) * Math.pow(yMovement, 2), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband((xMovement > 0 ? 1 : -1) * Math.pow(xMovement, 2), OIConstants.kDriveDeadband),
        (this.matchesDirectionWithinTolerance(angle, this.direction) ? 0 : calculateAngularSpeed(angle, direction)),//(angle > 0 ? -.5 : .5),
        true, true);
  }
  
  @Override
  public boolean isFinished()
  {
    return this.matchesDirection || -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband) == 0;
  }
  
  
  private double convertToSmallAngle(double angle) {
    while(angle > 180) {
      angle -= 360;
    }
    while (angle < -180) {
      angle += 360;
    }
    return angle;
  }

  private boolean matchesDirectionWithinTolerance(double angle, Direction direction) {
    double desiredAngle = calculateDesiredAngle(angle, direction);
    return desiredAngle + ANGLE_OFFSET > angle && desiredAngle - ANGLE_OFFSET <= angle;
  }

  private double calculateAngularSpeed(double angle, Direction direction) {
    // Scale down speed when we get closer to the target angle
    double calculatedSpeed = -1*(angle-calculateDesiredAngle(angle, direction))/180; 
    // Calculate sign of desired speed
    double sign = calculatedSpeed > 0 ? 1 : -1;
    // Between 10% and 30% calculated speed, set speed to 30%
    // When less than 10% calculated speed, set speed to 10% to prevent overshooting
    return sign*calculatedSpeed > .3 ? calculatedSpeed : (sign*calculatedSpeed >.1 ? sign*.3 : sign*.1);
  }

  private double calculateDesiredAngle(double angle, Direction direction) {
    double desiredAngle = 0;
    switch (direction) {
      case FORWARD:
        desiredAngle = 0;
        break;
      case LEFT:
        desiredAngle = 90;
        break;
      case RIGHT:
        desiredAngle = -90;
        break;
      case BACKWARD:
        desiredAngle = 180;
        angle = angle < 0 ? angle : angle + 360;
        break;
    }
    System.out.println("Desired angle: " + desiredAngle);
    return desiredAngle;
  }
}


  
