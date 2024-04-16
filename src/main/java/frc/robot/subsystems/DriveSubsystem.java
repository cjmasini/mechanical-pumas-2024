// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  public enum Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
  }

  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeftModule = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRightModule = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeftModule = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRightModule = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonGyroCanId);

  // PID Controller for orientation to supplied angle
  public final PIDController orientationController;
  
  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDirection = 0.0;
  private double currentTranslationMagnitude = 0.0;

  private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotationalLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double previousTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-1 * gyro.getAngle()),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      });

  public DriveSubsystem() {
      AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(AutoConstants.kPYController, 0.0, 0.0), // Rotation PID constants
                    4.8, // Max module speed, in m/s
                    Units.inchesToMeters(Math.sqrt(Math.pow(30, 2)+Math.pow(30,2))/2), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    orientationController = new PIDController(0, 0, 0); // TODO: Need to tune p, i and d
    orientationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(-1 * gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose (x / y coordinates and rotation)
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(-1 * gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot while it adjusts to a specified orientation. 
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param direction     Direction to orient front of robot towards.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void driveAndOrient(double xSpeed, double ySpeed, Direction direction, boolean rateLimit) {
    this.driveAndOrient(xSpeed, ySpeed, this.convertToSmallAngle(this.calculateDesiredAngle(direction), rateLimit);
  }

    /**
   * Method to drive the robot while it adjusts to a specified orientation. 
   *
   * @param xSpeed            Speed of the robot in the x direction (forward).
   * @param ySpeed            Speed of the robot in the y direction (sideways).
   * @param targetHeading     Target heading (angle) robot should face
   * @param rateLimit         Whether to enable rate limiting for smoother control.
   */
  public void driveAndOrient(double xSpeed, double ySpeed, double targetHeading, boolean rateLimit) {
    double currentHeading = this.convertToSmallAngle(this.getHeading());
    double targetHeading = this.convertToSmallAngle(targetHeading);
    
    // The left stick controls translation of the robot.
    // Automatically turn to face the supplied heading
    this.drive(
        xSpeed,
        ySpeed,
        //(this.matchesDirectionWithinTolerance(currentHeading, direction) ? 0 : calculateAngularVelocity(currentHeading, targetHeading)),
        calculatePIDAngularVelocity(currentHeading, targetHeading),
        true, 
        rateLimit);
  }

  /**
   * Call the drive function with a chassis speeds input
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    this.drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false, false);
  }

  /**
   * Get the reletive robot speeds for the robot
   * 
   * @return ChassisSpeeds containing states for each swerve module
   */
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeftModule.getState(),
                                                           frontRightModule.getState(),
                                                           rearLeftModule.getState(),
                                                           rearRightModule.getState());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {



    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMagnitude != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMagnitude);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - previousTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDirection);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDirection = SwerveUtils.StepTowardsCircular(currentTranslationDirection, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMagnitude > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
        }
        else {
          currentTranslationDirection = SwerveUtils.WrapAngle(currentTranslationDirection + Math.PI);
          currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDirection = SwerveUtils.StepTowardsCircular(currentTranslationDirection, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
      }
      previousTime = currentTime;
      
      xSpeedCommanded = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
      ySpeedCommanded = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
      currentRotation = rotationalLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-1 * gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    rearLeftModule.setDesiredState(swerveModuleStates[2]);
    rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    rearLeftModule.setDesiredState(desiredStates[2]);
    rearRightModule.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    rearLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-1 * gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Follow path planner path
   * 
   * @param pathName path planner path to follow
   * @return command to follow that path
   */
  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(AutoConstants.kPYController, 0.0, 0.0), // Rotation PID constants
                    4.8, // Max module speed, in m/s
                    Units.inchesToMeters(Math.sqrt(Math.pow(30, 2)+Math.pow(30,2))/2), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Converts an angle into a coterminal angle between -180 and 180 degrees
   * 
   * @param angle The angle to convert
   * @return A coterminal angle between -180 and 180 degrees
   */
  private double convertToSmallAngle(double angle) {
    while(angle > 180) {
      angle -= 360;
    }
    while (angle < -180) {
      angle += 360;
    }
    return angle;
  }

  /**
   * Checks whether an angle matches the desired diretion within a tolerance of 2 degrees
   * 
   * @param angle The angle to verify matching
   * @param direction The direction the angle should match
   * @return boolean representing whether the angle matches the supplied direction
   */
  private boolean matchesDirectionWithinTolerance(double angle, Direction direction) {
    double desiredAngle = calculateDesiredAngle(direction);
    return desiredAngle + DriveConstants.DRIVE_AND_ORIENT_ANGLE_TOLERANCE > angle && desiredAngle - DriveConstants.DRIVE_AND_ORIENT_ANGLE_TOLERANCE <= angle;
  }

  /**
   * Calculates the angular velocity to send to the drive method in order to rotate towards the supplied angle
   * Slows angular velocity as robot approaches target heading to avoid overshooting
   * 
   * @param currentHeading The current angle of the robot
   * @param targetHeading The direction the robot should face
   * @return The angular velocity required to rotate the robot to the desired heading
   */
  private double calculateAngularVelocity(double currentHeading, double targetHeading) {
    // Scale down speed when we get closer to the target angle
    double calculatedSpeed = -1*(currentHeading-targetHeading)/180; 
    // Calculate sign of desired speed
    double sign = calculatedSpeed > 0 ? 1 : -1;
    // Between 10% and 30% calculated speed, set speed to 30%
    // When less than 10% calculated speed, set speed to 10% to prevent overshooting
    return sign*calculatedSpeed > .3 ? calculatedSpeed : (sign*calculatedSpeed >.1 ? sign*.3 : sign*.1);
  }

    /**
   * Calculates the angular velocity to send to the drive method in order to rotate towards the supplied angle using a PID Controller
   * 
   * @param currentHeading The current angle of the robot
   * @param targetHeading The direction the robot should face
   * @return The angular velocity required to rotate the robot to the desired heading
   */
  private double calculatePIDAngularVelocity(double currentHeading, double targetHeading) {
    return this.orientationController.calculate(currentHeading, targetHeading);
  }

  /**
   * Calculate the desired angle coresponding with the supplied direction
   * 
   * @param angle The angle the robot is currently facing
   * @param direction The direction the robot should face
   * @return The angle corresponding to the desired direction
   */
  private double calculateDesiredAngle(Direction direction) {
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
        desiredAngle = angle < 0 ? -180 : 180;
        break;
    }
    return desiredAngle;
  }
}
