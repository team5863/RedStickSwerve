// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDriveTrain;

public class LimeAim extends Command {
  /** Creates a new LimeSwerve. */
     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDriveTrain m_driveTrain;
    private final Supplier<Double> getXspeed;
    private final Supplier<Double> getYspeed;
    private final Supplier<Double> getRotation;
    private final Supplier<Boolean> fieldRelative;
    private final SlewRateLimiter xLimiter, yLimiter, rotationLimiter;
  public LimeAim(SwerveDriveTrain m_driveTrain, Supplier<Double> getXspeed, Supplier<Double> getYspeed, Supplier<Double> getRotation, Supplier<Boolean> fieldRelative) {

    addRequirements(m_driveTrain);
    this.m_driveTrain = m_driveTrain;
    this.getXspeed = getXspeed;
    this.getYspeed = getYspeed;
    this.getRotation = getRotation;
    this.fieldRelative = fieldRelative;
    this.xLimiter = new SlewRateLimiter(Constants.SwerveConstants.DriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Constants.SwerveConstants.DriveMaxAccelerationUnitsPerSecond);
    this.rotationLimiter = new SlewRateLimiter(Constants.SwerveConstants.DriveMaxAngularAccelerationUnitsPerSecond);
    
  }

 
  @Override
  public void initialize() {}

  double kPLimeLightAim(){    
      double kP = .016554;
      double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
  
      targetingAngularVelocity *= Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;
  
      targetingAngularVelocity *= -1.0;
  
      return targetingAngularVelocity;
      
  }

  @Override
  public void execute() {

    double xSpeed = getXspeed.get();
    double ySpeed = getYspeed.get();
    double rotationSpeed = getRotation.get();

        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.limelightdeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.limelightdeadband ? ySpeed : 0.0;
        rotationSpeed = Math.abs(rotationSpeed) > Constants.OIConstants.limelightdeadband ? rotationSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.SwerveConstants.maxSpeed;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.SwerveConstants.maxSpeed;
        

  
    ChassisSpeeds chassisSpeeds;

      if(LimelightHelpers.getTV("limelight") == true){
        rotationSpeed = -kPLimeLightAim();
      }else{
        rotationSpeed = rotationLimiter.calculate(rotationSpeed)
        * Constants.SwerveConstants.DriveMaxAngularAccelerationUnitsPerSecond;
      }
            // Relative to robot
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

        SwerveModuleState[] moduleStates = Constants.SwerveConstants.DriveKinematics.toSwerveModuleStates(chassisSpeeds);

        
        m_driveTrain.setModuleStates(moduleStates);
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
