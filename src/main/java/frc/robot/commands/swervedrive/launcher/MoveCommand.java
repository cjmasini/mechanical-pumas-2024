package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Direction;

/**
 * Command for note intake
 */
public class MoveCommand extends Command 
{
  private CommandXboxController driverXbox;
  private DriveSubsystem driveSubsystem;
  private boolean fieldRelative = true;

  private static final double SPEED_REGULATOR = .25; 
  /**
   * Command for moving the robot using the swerve drive modules
   *
   * @param drivetrain  The drive subsystem.
   * @param driverXbox The xbox controller for the robot
   */
  public MoveCommand(DriveSubsystem drivetrain, CommandXboxController driverXbox)
  {
      this.driveSubsystem = drivetrain;
      this.driverXbox = driverXbox;
      addRequirements(this.driveSubsystem);
  }

  @Override
  public void execute() {
    // Trigger mappings for fine tuned adjustments using the d-pad
    if (driverXbox.povLeft().getAsBoolean()) {
      driveSubsystem.drive(0, .2, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), false, true);
    } else if (driverXbox.povRight().getAsBoolean()) {
      driveSubsystem.drive(0, -.2, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), false, true);
    } else if (driverXbox.povUp().getAsBoolean()) {
      driveSubsystem.drive(.2, 0, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), false, true);
    } else if (driverXbox.povDown().getAsBoolean()) {
      driveSubsystem.drive(-0.2, 0, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), false, true);
    } else if (driverXbox.povUpLeft().getAsBoolean()) {
      driveSubsystem.drive(.14, .14, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), false, true);
    } else if (driverXbox.povUpRight().getAsBoolean()) {
      driveSubsystem.drive(.14, -.14, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), false, true);
    } else if (driverXbox.povDownLeft().getAsBoolean()) {
      driveSubsystem.drive(-.14, .14, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), false, true);
    } else if (driverXbox.povDownRight().getAsBoolean()) {
      driveSubsystem.drive(-.14, -.14, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), false, true);
    } else {
      double yMovement = driverXbox.getLeftY();
      double xMovement = driverXbox.getLeftX();

      // Trigger mappings for driving while orienting to a supplied direction
      if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.x().getAsBoolean()) {
        this.driveSubsystem.driveAndOrient(
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(yMovement, 3), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(xMovement, 3), OIConstants.kDriveDeadband),
            Direction.RIGHT,
             true);
      } else if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.y().getAsBoolean()) {
        this.driveSubsystem.driveAndOrient(
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(yMovement, 3), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(xMovement, 3), OIConstants.kDriveDeadband),
            Direction.BACKWARD,
             true);
      } else if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.b().getAsBoolean()) {
        this.driveSubsystem.driveAndOrient(
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(yMovement, 3), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(xMovement, 3), OIConstants.kDriveDeadband),
            Direction.LEFT,
             true);
      } else if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.a().getAsBoolean()) {
        this.driveSubsystem.driveAndOrient(
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(yMovement, 3), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(xMovement, 3), OIConstants.kDriveDeadband),
            Direction.FORWARD,
             true);
      } else {
        // Default joystick controlled swerve
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        this.driveSubsystem.drive(
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(yMovement, 3), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(SPEED_REGULATOR * Math.pow(xMovement, 3), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband),
            fieldRelative, true);
      }
    }
  }
  
  @Override
  public boolean isFinished()
  {
    return false;
  }

  public boolean getFieldReletive() {
    return this.fieldRelative;
  }

  public void toggleFieldReletive() {
    this.fieldRelative = !this.fieldRelative;
  }
}
