// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
   private final CANSparkMax driveMotor;
   private final CANSparkMax angleMotor;
   private final boolean absoluteEncoderReversed;
   private final double absoluteEncoderOffsetRad;
   
   
   private final RelativeEncoder m_driveEncoder;
   private final RelativeEncoder m_angleEncoder;
   private final CANcoder m_absoluteEncoder;

   public  final PIDController m_anglePidController,
                               m_drivePidController;

  public SwerveModule(int driveMotorID, int angleMotorID, int encoderID, boolean driveMotorReversed, boolean angleMotorReversed,
    double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    
      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      
      driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
      angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

      driveMotor.setInverted(driveMotorReversed);
      angleMotor.setInverted(angleMotorReversed);

      driveMotor.setIdleMode(IdleMode.kBrake);
      angleMotor.setIdleMode(IdleMode.kBrake);

      m_driveEncoder = driveMotor.getEncoder();
      m_angleEncoder = angleMotor.getEncoder();
      m_absoluteEncoder = new CANcoder(encoderID);

      m_driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveEncoderRot2Meter);
      m_driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveEncoderRPM2MeterPerSec);
      m_angleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleEncoderRot2Rad);
      m_angleEncoder.setVelocityConversionFactor(Constants.SwerveConstants.angleEncoderRPM2RadPerSec);

      m_anglePidController = new PIDController(Constants.SwerveConstants.anglekP,//SmartDashboard.getNumber("AnglekP", 0),
                                               Constants.SwerveConstants.anglekI,
                                               Constants.SwerveConstants.anglekD
                                              );

      m_anglePidController.setTolerance(0.01);
      
      m_drivePidController = new PIDController(Constants.SwerveConstants.drivekP,
                                               Constants.SwerveConstants.drivekI,
                                               Constants.SwerveConstants.drivekD
                                              );

      m_anglePidController.enableContinuousInput(-Math.PI, Math.PI);
      
      m_drivePidController.setTolerance(0.01);

      resetEncoders();

  }

public double getDrivePosition() {
 
  return m_driveEncoder.getPosition();

}

public double getAnglePosition() {

  return m_angleEncoder.getPosition();
} 

public double getDriveVelocity() {

  return m_driveEncoder.getVelocity();
}

public double getAngleVelocity() {

  return m_angleEncoder.getVelocity();

}

public double getAbsolutePosition() {


  double angle = m_absoluteEncoder.getAbsolutePosition().getValueAsDouble();
  
  angle *= (2*Math.PI);
  

    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
}

public void resetEncoders() {
  m_driveEncoder.setPosition(0);

  m_angleEncoder.setPosition(getAbsolutePosition());
}

public SwerveModuleState getState() {

  return new SwerveModuleState( getDriveVelocity(), new Rotation2d(getAnglePosition()));
}

public SwerveModulePosition getPosition() {
  return new SwerveModulePosition( getDrivePosition(), new Rotation2d(getAnglePosition()));
}

public void setDesiredState( SwerveModuleState state) {


  if(Math.abs(state.speedMetersPerSecond) < 0.001) {
    stop();
    return;
  }

  state = SwerveModuleState.optimize(state, getState().angle);
  //driveMotor.set(m_drivePidController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
  driveMotor.set(-state.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed);
  angleMotor.set(m_anglePidController.calculate(getAnglePosition(), state.angle.getRadians()));
  
 
  
}

public void stop() {
  driveMotor.set(0);
  angleMotor.set(0);
}
  @Override
  public void periodic() {
    
   m_angleEncoder.setPosition(getAbsolutePosition());
    SmartDashboard.getNumber("anglePIDAngle", m_anglePidController.getVelocityError());
    SmartDashboard.getNumber("anglePIDPosition", m_anglePidController.getPositionError());
  
   
  }
}
