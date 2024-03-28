// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
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
import frc.robot.subsystems.LauncherSubsystem;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final DriveSubsystem drivetrain = new DriveSubsystem();

  // The robot's shooter and intake mechanisms are defined here
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  
  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final Ultrasonic ultrasonicSensor = new Ultrasonic(DriveConstants.ultrasonicPingChannel, DriveConstants.ultrasonicEchoChannel);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    DoubleSupplier ultrasonicDistance = () -> this.ultrasonicSensor.getRangeMM();

    MoveCommand moveCommand = new MoveCommand(this.drivetrain, driverXbox);
    drivetrain.setDefaultCommand(moveCommand);
    
    IntakeCommand intakeCommand = new IntakeCommand(launcherSubsystem, ultrasonicDistance);
    driverXbox.rightBumper().onTrue(intakeCommand.withTimeout(10));

    double launchSpeed = .75;
    SmartDashboard.putNumber("launchSpeed", launchSpeed);
    AmpCommand ampCommand = new AmpCommand(launcherSubsystem, launchSpeed);
    driverXbox.leftBumper().onTrue(ampCommand.withTimeout(10));
    
    LowerCommand lowerCommand = new LowerCommand(launcherSubsystem);
    driverXbox.a().and(driverXbox.rightTrigger().negate()).onTrue(lowerCommand.withTimeout(10));

    RaiseCommand raiseCommand = new RaiseCommand(launcherSubsystem);
    driverXbox.x().and(driverXbox.rightTrigger().negate()).onTrue(raiseCommand.withTimeout(10));

    ReverseCommand reverseCommand = new ReverseCommand(launcherSubsystem);
    driverXbox.b().and(driverXbox.rightTrigger().negate()).onTrue(reverseCommand.withTimeout(10));

    InstantCommand toggleFieldReletive = new InstantCommand(() -> moveCommand.toggleFieldReletive());
    driverXbox.y().and(driverXbox.rightTrigger().negate()).onTrue(toggleFieldReletive);
    
    SpeakerCommand speakerCommand = new SpeakerCommand(launcherSubsystem);
    driverXbox.leftTrigger().onTrue(speakerCommand.withTimeout(10));

    CancelCommand cancelCommand = new CancelCommand(launcherSubsystem, ultrasonicDistance);
    driverXbox.rightTrigger().onTrue(cancelCommand.withTimeout(10));

    InstantCommand resetGyro = new InstantCommand(() -> this.drivetrain.zeroHeading());
    driverXbox.rightStick().onTrue(resetGyro);

    NamedCommands.registerCommand("zeroGyro", resetGyro);
    IntakeCommand autoIntakeCommand = new IntakeCommand(launcherSubsystem, ultrasonicDistance);
    NamedCommands.registerCommand("intakeCommand", autoIntakeCommand.withTimeout(3));
    SpeakerCommand autoSpeakerCommand = new SpeakerCommand(launcherSubsystem);
    NamedCommands.registerCommand("speakerCommand", autoSpeakerCommand.withTimeout(2));
    
    this.autoChooser = AutoBuilder.buildAutoChooser("Center Note Score");
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  public Command getAutonomousCommand() {
    return new AutonomousCommand(drivetrain);
  }
}