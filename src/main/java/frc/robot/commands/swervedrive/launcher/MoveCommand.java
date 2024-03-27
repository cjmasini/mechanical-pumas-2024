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
  // private boolean matchesDirection = false;


  /**
   * Command for intaking notes
   *
   * @param launcher  The launcher subsystem.
   * @param ultrasonicDistance Distance from sensor to note in mm
   */
  public MoveCommand(DriveSubsystem driveSubsystem, CommandXboxController driverXbox)
  {
      this.driveSubsystem = driveSubsystem;
      this.driverXbox = driverXbox;
      addRequirements(this.driveSubsystem);
  }

  @Override
  public void execute() {
    if (driverXbox.povLeft().getAsBoolean()) {
      driveSubsystem.drive(0, .2, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), fieldRelative, true);
    } else if (driverXbox.povRight().getAsBoolean()) {
      driveSubsystem.drive(0, -.2, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), fieldRelative, true);
    } else if (driverXbox.povUp().getAsBoolean()) {
      driveSubsystem.drive(.2, 0, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), fieldRelative, true);
    } else if (driverXbox.povDown().getAsBoolean()) {
      driveSubsystem.drive(-0.2, 0, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), fieldRelative, true);
    } else if (driverXbox.povUpLeft().getAsBoolean()) {
      driveSubsystem.drive(.14, .14, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), fieldRelative, true);
    } else if (driverXbox.povUpRight().getAsBoolean()) {
      driveSubsystem.drive(.14, -.14, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), fieldRelative, true);
    } else if (driverXbox.povDownLeft().getAsBoolean()) {
      driveSubsystem.drive(-.14, .14, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), fieldRelative, true);
    } else if (driverXbox.povDownRight().getAsBoolean()) {
      driveSubsystem.drive(-.14, -.14, -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kDriveDeadband), fieldRelative, true);
    } else {

    double yMovement = driverXbox.getLeftY();
    double xMovement = driverXbox.getLeftX();

    if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.b().getAsBoolean()) {
      this.driveSubsystem.driveAndOrient(
          -MathUtil.applyDeadband((yMovement > 0 ? 1 : -1) * Math.pow(yMovement, 2), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband((xMovement > 0 ? 1 : -1) * Math.pow(xMovement, 2), OIConstants.kDriveDeadband),
          Direction.RIGHT,
           true);
    } else if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.a().getAsBoolean()) {
      this.driveSubsystem.driveAndOrient(
          -MathUtil.applyDeadband((yMovement > 0 ? 1 : -1) * Math.pow(yMovement, 2), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband((xMovement > 0 ? 1 : -1) * Math.pow(xMovement, 2), OIConstants.kDriveDeadband),
          Direction.BACKWARD,
           true);
    } else if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.x().getAsBoolean()) {
      this.driveSubsystem.driveAndOrient(
          -MathUtil.applyDeadband((yMovement > 0 ? 1 : -1) * Math.pow(yMovement, 2), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband((xMovement > 0 ? 1 : -1) * Math.pow(xMovement, 2), OIConstants.kDriveDeadband),
          Direction.LEFT,
           true);
    } else if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.y().getAsBoolean()) {
      this.driveSubsystem.driveAndOrient(
          -MathUtil.applyDeadband((yMovement > 0 ? 1 : -1) * Math.pow(yMovement, 2), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband((xMovement > 0 ? 1 : -1) * Math.pow(xMovement, 2), OIConstants.kDriveDeadband),
          Direction.FORWARD,
           true);
    } else {
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      this.driveSubsystem.drive(
          -MathUtil.applyDeadband((yMovement > 0 ? 1 : -1) * Math.pow(yMovement, 2), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband((xMovement > 0 ? 1 : -1) * Math.pow(xMovement, 2), OIConstants.kDriveDeadband),
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


  
