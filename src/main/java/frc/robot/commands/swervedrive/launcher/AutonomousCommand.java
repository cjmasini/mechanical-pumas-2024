package frc.robot.commands.swervedrive.launcher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  public AutonomousCommand(DriveSubsystem driveSubsystem, LauncherSubsystem launcher, int mode)
  {
    this.launcher = launcher;
    this.drivetrain = driveSubsystem;
    if (mode == 1)
    {
      double timeUnit = .5;
      double speed = .5;

      Command zeroGyroCommand = new InstantCommand(() -> this.drivetrain.zeroHeading());
      SpeakerCommand speakerCommand = new SpeakerCommand(launcher);
      WaitCommand waitCommand1 = new WaitCommand(1.5);
      CancelCommand cancelCommand = new CancelCommand(launcher);
      Command moveForward1 = Commands.startEnd(() -> this.drivetrain.drive(speed, 0, 0, true, true), () -> this.drivetrain.drive(0, 0, 0, true, true)).withTimeout(2*timeUnit);
      Command intakeCommand = new RunCommand(this::intakeNote, driveSubsystem).withTimeout(2);
      CancelCommand cancelCommand2 = new CancelCommand(launcher);
      WaitCommand waitCommand2 = new WaitCommand(1.5);
      CancelCommand cancelCommand3 = new CancelCommand(launcher);

      this.addCommands(zeroGyroCommand, speakerCommand, waitCommand1, cancelCommand, moveForward1, intakeCommand, cancelCommand2,waitCommand2, cancelCommand3);

    } else if (mode == 2) {
      double timeUnit = .4;
      double speed = .5;

      Command zeroGyroCommand = new InstantCommand(() -> this.drivetrain.zeroHeading());
      WaitCommand waitCommand0 = new WaitCommand(0.2);
      SpeakerCommand speakerCommand = new SpeakerCommand(launcher);
      WaitCommand waitCommand1 = new WaitCommand(1.5);
      CancelCommand cancelCommand = new CancelCommand(launcher);
      Command moveForward1 = Commands.startEnd(() -> this.drivetrain.drive(speed, 0, 0, true, true), () -> this.drivetrain.drive(0, 0, 0, true, true)).withTimeout(1.5*timeUnit);
      Command intakeCommand = new RunCommand(this::intakeNote, driveSubsystem).withTimeout(2);
      CancelCommand cancelCommand2 = new CancelCommand(launcher);
      WaitCommand waitCommand2 = new WaitCommand(1.5);
      CancelCommand cancelCommand3 = new CancelCommand(launcher);
      Command moveBackward = Commands.startEnd(() -> this.drivetrain.drive(-1*speed, 0, 0, true, true), () -> this.drivetrain.drive(0, 0, 0, true, true)).withTimeout(3*timeUnit);
      SpeakerCommand speakerCommand2 = new SpeakerCommand(launcher);
      WaitCommand waitCommand3 = new WaitCommand(.5);
      Command moveForward3 = Commands.startEnd(() -> this.drivetrain.drive(speed, 0, 0, true, true), () -> this.drivetrain.drive(0, 0, 0, true, true)).withTimeout(3*timeUnit);

      this.addCommands(zeroGyroCommand, waitCommand0, speakerCommand, waitCommand1, cancelCommand, moveForward1, intakeCommand, cancelCommand2, moveBackward, waitCommand3, speakerCommand2, waitCommand2, cancelCommand3, moveForward3);
    } else {
      double timeUnit = .4;
      double speed = .5;

      Command zeroGyroCommand = new InstantCommand(() -> this.drivetrain.zeroHeading());
      WaitCommand waitCommand0 = new WaitCommand(0.2);
      SpeakerCommand speakerCommand = new SpeakerCommand(launcher);
      WaitCommand waitCommand1 = new WaitCommand(1.5);
      CancelCommand cancelCommand = new CancelCommand(launcher);
      Command moveForward1 = Commands.startEnd(() -> this.drivetrain.drive(speed, 0, 0, true, true), () -> this.drivetrain.drive(0, 0, 0, true, true)).withTimeout(1.5*timeUnit);
      Command intakeCommand = new RunCommand(this::intakeNote, driveSubsystem).withTimeout(2);
      CancelCommand cancelCommand2 = new CancelCommand(launcher);
      WaitCommand waitCommand2 = new WaitCommand(1.5);
      CancelCommand cancelCommand3 = new CancelCommand(launcher);
      Command moveBackward = Commands.startEnd(() -> this.drivetrain.drive(-1*speed, 0, 0, true, true), () -> this.drivetrain.drive(0, 0, 0, true, true)).withTimeout(3*timeUnit);
      SpeakerCommand speakerCommand2 = new SpeakerCommand(launcher);
      WaitCommand waitCommand3 = new WaitCommand(.5);
      Command moveForward3 = Commands.startEnd(() -> this.drivetrain.drive(speed, 0, 0, true, true), () -> this.drivetrain.drive(0, 0, 0, true, true)).withTimeout(3*timeUnit);

      this.addCommands(zeroGyroCommand, waitCommand0, speakerCommand, waitCommand1, cancelCommand, moveForward1, intakeCommand, cancelCommand2, moveBackward, waitCommand3, speakerCommand2, waitCommand2, cancelCommand3, moveForward3);
    }


  }

  private void intakeNote() {
    this.launcher.setAssistSpeed(0);
    this.launcher.setLoaderSpeed(.5);
    this.launcher.setIntakeSpeed(.5);
  }

  private void stopIntakeNote() {
    this.launcher.setAssistSpeed(0);
    this.launcher.setLoaderSpeed(0);
    this.launcher.setIntakeSpeed(0);
  }
}


  
