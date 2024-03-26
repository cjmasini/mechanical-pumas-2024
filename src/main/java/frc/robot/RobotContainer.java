// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swervedrive.launcher.AmpCommand;
import frc.robot.commands.swervedrive.launcher.AutonomousCommand;
import frc.robot.commands.swervedrive.launcher.CancelCommand;
import frc.robot.commands.swervedrive.launcher.IntakeCommand;
import frc.robot.commands.swervedrive.launcher.LowerCommand;
import frc.robot.commands.swervedrive.launcher.MoveCommand;
import frc.robot.commands.swervedrive.launcher.RaiseCommand;
import frc.robot.commands.swervedrive.launcher.ReverseCommand;
import frc.robot.commands.swervedrive.launcher.SpeakerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private static final int PING_CHANNEL = 0;

  private static final int ECHO_CHANNEL = 1;

  // The robot's subsystems and commands are defined here
  // private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  //                                                                        "swerve/neo"));

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The robot's shooter and intake mechanisms are defined here
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  
  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final Ultrasonic ultrasonicSensor = new Ultrasonic(PING_CHANNEL, ECHO_CHANNEL);

  private boolean invertedDrive = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    System.out.println("In robot container");
    DoubleSupplier ultrasonicDistance = () -> this.ultrasonicSensor.getRangeMM();

    IntakeCommand intakeCommand = new IntakeCommand(launcherSubsystem, ultrasonicDistance);
    driverXbox.rightBumper().onTrue(intakeCommand.withTimeout(10));

    AmpCommand ampCommand = new AmpCommand(launcherSubsystem, .75);
    driverXbox.leftBumper().onTrue(ampCommand.withTimeout(10));

    AmpCommand ampCommand2 = new AmpCommand(launcherSubsystem, .65);
    driverXbox.y().onTrue(ampCommand2.withTimeout(10));

    LowerCommand lowerCommand = new LowerCommand(launcherSubsystem);
    driverXbox.a().onTrue(lowerCommand.withTimeout(10));

    RaiseCommand raiseCommand = new RaiseCommand(launcherSubsystem);
    driverXbox.x().onTrue(raiseCommand.withTimeout(10));

    SpeakerCommand speakerCommand = new SpeakerCommand(launcherSubsystem);
    driverXbox.leftTrigger().onTrue(speakerCommand.withTimeout(10));

    CancelCommand cancelCommand = new CancelCommand(launcherSubsystem, ultrasonicDistance);
    driverXbox.rightTrigger().onTrue(cancelCommand.withTimeout(10));

    ReverseCommand reverseCommand = new ReverseCommand(launcherSubsystem);
    driverXbox.b().onTrue(reverseCommand.withTimeout(10));

     // Configure default commands
     m_robotDrive.setDefaultCommand(new MoveCommand(m_robotDrive, driverXbox));
  }
  
  public Command getAutonomousCommand() {
    
      return new AutonomousCommand(m_robotDrive);
  }
}
