// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class SwerveDriveTrain extends SubsystemBase {
  /** Creates a new SwerveDriveTrain. */

  private final SwerveModule FrontLeft = new SwerveModule(
    Constants.FrontLeftModule.driveID, Constants.FrontLeftModule.angleID, Constants.FrontLeftModule.encoderID,
    Constants.FrontLeftModule.driveMotorReversed, Constants.FrontLeftModule.angleMotorReversed,
    Constants.FrontLeftModule.absoluteEncoderOffset, Constants.FrontLeftModule.absoluteEncoderReversed);

  private final SwerveModule FrontRight = new SwerveModule(
    Constants.FrontRightModule.driveID, Constants.FrontRightModule.angleID, Constants.FrontRightModule.encoderID,
    Constants.FrontRightModule.driveMotorReversed, Constants.FrontRightModule.angleMotorReversed,
    Constants.FrontRightModule.absoluteEncoderOffset, Constants.FrontRightModule.absoluteEncoderReversed);

  private final SwerveModule BackLeft = new SwerveModule(
    Constants.BackLeftModule.driveID, Constants.BackLeftModule.angleID, Constants.BackLeftModule.encoderID,
    Constants.BackLeftModule.driveMotorReversed, Constants.BackLeftModule.angleMotorReversed,
    Constants.BackLeftModule.absoluteEncoderOffset, Constants.BackLeftModule.absoluteEncoderReversed);

  private final SwerveModule BackRight = new SwerveModule(
    Constants.BackRightModule.driveID, Constants.BackRightModule.angleID, Constants.BackRightModule.encoderID,
    Constants.BackRightModule.driveMotorReversed, Constants.BackRightModule.angleMotorReversed,
    Constants.BackRightModule.absoluteEncoderOffset, Constants.BackRightModule.absoluteEncoderReversed);
  

  private final Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.gyroPort);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.SwerveConstants.DriveKinematics,
            new Rotation2d(0), new SwerveModulePosition[] {
              FrontLeft.getPosition(), FrontRight.getPosition(),
              BackLeft.getPosition(), BackRight.getPosition()
            });

  public SwerveDriveTrain() {
        new Thread( () -> {
            try {
              Thread.sleep(1000);
              zeroHeading();
            } catch (Exception e) {
            }
        }).start();

         AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::pathPlannerRobotDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    3, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
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


  public void zeroHeading() {
    gyro.setYaw(0);
  }

  public void setHeading(double startRotation) {

    gyro.setYaw(startRotation);
  }

  public double getHeading() {

     double heading = gyro.getYaw().getValueAsDouble();

    return Math.IEEEremainder(heading, 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public  Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    
    ChassisSpeeds state = Constants.SwerveConstants.DriveKinematics.toChassisSpeeds(
      FrontLeft.getState(),
      FrontRight.getState(),
      BackLeft.getState(),
      BackRight.getState());

    return state;
  }
  
  public void pathPlannerRobotDrive(ChassisSpeeds speeds){

    SwerveModuleState[] moduleStates = Constants.SwerveConstants.DriveKinematics.toSwerveModuleStates(speeds);

    setModuleStates(moduleStates);
  }
  
  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(
      getRotation2d(),
      new SwerveModulePosition[] {
              FrontLeft.getPosition(), FrontRight.getPosition(),
              BackLeft.getPosition(), BackRight.getPosition()},
      pose
      );
  }

  public void stopModules() {
    FrontLeft.stop();
    FrontRight.stop();
    BackLeft.stop();
    BackRight.stop();
  }

  public void setModuleStates( SwerveModuleState[] desiredStates) {
    //SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
    FrontLeft.setDesiredState(desiredStates[0]);
    FrontRight.setDesiredState(desiredStates[1]);
    BackLeft.setDesiredState(desiredStates[2]);
    BackRight.setDesiredState(desiredStates[3]);

    SmartDashboard.putNumber("FrontLeftModule", (desiredStates[0].speedMetersPerSecond / Constants.SwerveConstants.maxSpeed));
    SmartDashboard.putNumber("FrontRightModule", (desiredStates[1].speedMetersPerSecond / Constants.SwerveConstants.maxSpeed));
    SmartDashboard.putNumber("BackLeftModule", (desiredStates[2].speedMetersPerSecond / Constants.SwerveConstants.maxSpeed));
    SmartDashboard.putNumber("BackRightModule", (desiredStates[3].speedMetersPerSecond / Constants.SwerveConstants.maxSpeed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
              FrontLeft.getPosition(), FrontRight.getPosition(),
              BackLeft.getPosition(), BackRight.getPosition()
            }
          );
    SmartDashboard.putNumber( "Robot Heading", getHeading());
    SmartDashboard.putNumber( "Target Angle_FL", FrontLeft.m_anglePidController.getSetpoint());
    SmartDashboard.putNumber("Angle in Radians_FL", FrontLeft.getAnglePosition());
    SmartDashboard.putNumber("Target Angle_FR", FrontRight.m_anglePidController.getSetpoint());
    SmartDashboard.putNumber("Angle in Radians_FR", FrontRight.getAbsolutePosition());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putNumber("Target Velocity", FrontLeft.m_drivePidController.getSetpoint());
    SmartDashboard.putNumber(" Robot Velocity", FrontLeft.getDriveVelocity());
    //SmartDashboard.putNumber("driveEncoderRPM2MeterPerSec", Constants.SwerveConstants.driveEncoderRPM2MeterPerSec);
    //SmartDashboard.putNumber("angleEncoderRPM2RadPerSec", Constants.SwerveConstants.angleEncoderRPM2RadPerSec);
  }
}


