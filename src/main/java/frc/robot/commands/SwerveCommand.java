// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveTrain;
import java.util.function.Supplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDriveTrain m_driveTrain;
    private final Supplier<Double> getXspeed;
    private final Supplier<Double> getYspeed;
    private final Supplier<Double> getRotation;
    private final Supplier<Boolean> fieldRelative;
    private final SlewRateLimiter xLimiter, yLimiter, rotationLimiter;
  public SwerveCommand(SwerveDriveTrain m_driveTrain, Supplier<Double> getXspeed, Supplier<Double> getYspeed, Supplier<Double> getRotation, Supplier<Boolean> fieldRelative) {

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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = getXspeed.get();
    double ySpeed = getYspeed.get();
    double rotationSpeed = getRotation.get();

    xSpeed = Math.abs(xSpeed) > Constants.OIConstants.deadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.deadband ? ySpeed : 0.0;
        rotationSpeed = Math.abs(rotationSpeed) > Constants.OIConstants.deadband ? rotationSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.SwerveConstants.maxSpeed;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.SwerveConstants.maxSpeed;
        rotationSpeed = rotationLimiter.calculate(rotationSpeed)
                * Constants.SwerveConstants.DriveMaxAngularAccelerationUnitsPerSecond;

      /*SwerveModuleState[] moduleStates =
        Constants.SwerveConstants.DriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(fieldRelative.get() 
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  xSpeed, ySpeed, rotationSpeed, m_driveTrain.getRotation2d())
              : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed),
            0.05));*/
              

    ChassisSpeeds chassisSpeeds;
        if (fieldRelative.get() ) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotationSpeed, m_driveTrain.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        }

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
