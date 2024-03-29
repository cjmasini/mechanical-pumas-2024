// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;


public class LauncherSubsystem extends SubsystemBase
{
  /**
   * Intake Motor
   */
  private final CANSparkMax intakeMotor;
  private static final int INTAKE_CONTROLLER_ID = 51;

  /**
   * Loading Wheel Motor
   */
  private final Spark loaderMotor;
  private static final int LOADER_CHANNEL_ID = 0;

  private final VictorSPX assistMotor;
  private static final int ASSIST_CAN_ID = 52;

  /**
   * Launch Motors
   */
  private final Spark leftLaunchMotor;
  private static final int LEFT_LAUNCH_CHANNEL_ID = 2;

  private final Spark rightLaunchMotor;
  private static final int RIGHT_LAUNCH_CHANNEL_ID = 4;

  /**
   * Left Elevator Motor 
   */
  private final Spark leftElevatorMotor;
  private static final int LEFT_ELEVATOR_CHANNEL_ID = 3;
  
  /**
   * Right Elevator Motor 
   */
  private final Spark rightElevatorMotor;
  private static final int RIGHT_ELEVATOR_CHANNEL_ID = 1;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public LauncherSubsystem()
  {
    this.setName("LauncherSubsytem");

    System.out.println("Setting up intake=============================================");
    this.intakeMotor = new CANSparkMax(INTAKE_CONTROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
    this.intakeMotor.restoreFactoryDefaults();
    this.intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    this.intakeMotor.setInverted(true);

    this.loaderMotor = new Spark(LOADER_CHANNEL_ID);
    loaderMotor.setInverted(true);

    this.assistMotor = new VictorSPX(ASSIST_CAN_ID);
    assistMotor.setInverted(true);

    this.leftLaunchMotor = new Spark(LEFT_LAUNCH_CHANNEL_ID);
    this.leftLaunchMotor.setInverted(false);

    this.rightLaunchMotor = new Spark(RIGHT_LAUNCH_CHANNEL_ID);
    this.rightLaunchMotor.setInverted(true);

    this.leftElevatorMotor = new Spark(LEFT_ELEVATOR_CHANNEL_ID);
    this.leftElevatorMotor.setInverted(true);

    this.rightElevatorMotor = new Spark(RIGHT_ELEVATOR_CHANNEL_ID);
    this.rightElevatorMotor.setInverted(true);
  }

  public void setIntakeSpeed(double intakeSpeed){
    this.intakeMotor.set(intakeSpeed);
  }
  public void setLoaderSpeed(double loaderSpeed){
    this.loaderMotor.set(loaderSpeed);;
  }
  public void setAssistSpeed(double assistSpeed){
    this.assistMotor.set(VictorSPXControlMode.PercentOutput, assistSpeed);
  }
  public void setLaunchSpeed(double launchSpeed){
    this.leftLaunchMotor.set(launchSpeed);
    this.rightLaunchMotor.set(launchSpeed);
  }
  public void setElevatorSpeed(double elevatorSpeed){
    this.rightElevatorMotor.set(elevatorSpeed);
    this.leftElevatorMotor.set(elevatorSpeed);
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }
}
