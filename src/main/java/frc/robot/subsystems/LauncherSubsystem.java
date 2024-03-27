// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

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

  /**
   * Launch Motor
   */
  private final Spark launchMotor;
  private static final int LAUNCH_CHANNEL_ID = 2;

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

    this.launchMotor = new Spark(LAUNCH_CHANNEL_ID);
    this.launchMotor.setInverted(false);

    this.leftElevatorMotor = new Spark(LEFT_ELEVATOR_CHANNEL_ID);
    this.leftElevatorMotor.setInverted(true);

    this.rightElevatorMotor = new Spark(RIGHT_ELEVATOR_CHANNEL_ID);
    this.rightElevatorMotor.setInverted(true);
  }



  public void setIntakeSpeed(double intakeSpeed){
    this.intakeMotor.set(intakeSpeed);
  }
  public double getIntakeSpeed(){
    return this.intakeMotor.get();
  }
  public void setLoaderSpeed(double loaderSpeed){
    this.loaderMotor.set(loaderSpeed);;
  }
  public double getLoaderSpeed(){
    return this.loaderMotor.get();
  }
  public void setLaunchSpeed(double launchSpeed){
    this.launchMotor.set(launchSpeed);
  }
  public double getLaunchSpeed(){
    return this.launchMotor.get();
  }
  public void setLeftElevatorSpeed(double elevatorSpeed){
    this.leftElevatorMotor.set(elevatorSpeed);;
  }
  public double getLeftElevatorSpeed(){
    return this.leftElevatorMotor.get();
  }
  public void setRightElevatorSpeed(double elevatorSpeed){
    this.rightElevatorMotor.set(elevatorSpeed);;
  }
  public double getRightElevatorSpeed(){
    return this.rightElevatorMotor.get();
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
