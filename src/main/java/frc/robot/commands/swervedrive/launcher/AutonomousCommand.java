package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Command for note intake
 */
public class AutonomousCommand extends SequentialCommandGroup 
{

  private final DriveSubsystem drivetrain;
  private final LauncherSubsystem launcher;
  
  /**
   * Command for autonomous mode
   *
   * @param drivetrain The drivetrain
   * @param launcher  The launcher subsystem.
   */
  public AutonomousCommand(DriveSubsystem driveSubsystem, LauncherSubsystem launcher)
  {
    this.launcher = launcher;
    this.drivetrain = driveSubsystem;

    Command zeroGyroCommand = new InstantCommand(() -> this.drivetrain.zeroHeading());
    SpeakerCommand speakerCommand = new SpeakerCommand(launcher);
    WaitCommand waitTwoSeconds = new WaitCommand(2);
    CancelCommand cancelCommand = new CancelCommand(launcher);
    Command moveForward1 = new RunCommand(() -> this.drivetrain.drive(.5, 0, 0, true, true), driveSubsystem).withTimeout(1);
    IntakeCommand intakeCommand = new IntakeCommand(launcher, null);
    Command moveForward2 = new RunCommand(() -> this.drivetrain.drive(.5, 0, 0, true, true), driveSubsystem).withTimeout(.5);
    Command moveBackward = new RunCommand(() -> this.drivetrain.drive(-.5, 0, 0, true, true), driveSubsystem).withTimeout(1.5);

    this.addCommands(zeroGyroCommand, speakerCommand, waitTwoSeconds, cancelCommand, moveForward1, intakeCommand, moveForward2, cancelCommand, moveBackward, speakerCommand, waitTwoSeconds, moveForward1);
    addRequirements(launcher);
  }
}


  
