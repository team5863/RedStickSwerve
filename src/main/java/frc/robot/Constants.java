// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
   public static class CommandConstants {
     

    public static class Shoot {
      public static final int shooterSparkMax = 4;
    }
  
    public final class Serialize {
      public static final int serializerSparkMax = 5;
    }
  
    public static class Intake {
      public static final int intakeSparkMax = 6;
    }
    public static class FeedWheel {
      public static final int feedWheelSpark = 7;
    }
   }
  
  public static class SwerveConstants {
    
    public static final double neoRPM = 5820;
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveMotorGearRatio = ( 1.0 / 8.14);
    public static final double angleMotorGearRatio = ( 1.0 / (150.0 / 7.0) );
    public static final double driveEncoderRot2Meter = driveMotorGearRatio * Math.PI *wheelDiameter;
    public static final double angleEncoderRot2Rad = angleMotorGearRatio * 2 * Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter / 60.0;
    public static final double angleEncoderRPM2RadPerSec = angleEncoderRot2Rad / 60.0;

    //public static final double drivekP = 0.00023;
    //public static final double drivekI = 0.0000002;
    //public static final double drivekD = 1.0;

    //test
    public static final double drivekP = 0.01;
    public static final double drivekI = 0.002;
    public static final double drivekD = 0.001;


    //test
    public static final double anglekP = 0.15;
    public static final double anglekI = 0.0;
    public static final double anglekD = 0.01;

    //public static final double anglekP = 0.15;
    //public static final double anglekI = 0.0;
    //public static final double anglekD = 0.01;

    
    public static final double maxSpeed = neoRPM * driveMotorGearRatio * (Math.PI*wheelDiameter) * (1.0/60.0);

    public static final double DriveMaxAccelerationUnitsPerSecond = 3;
    public static final double DriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final int gyroPort = 16;

     public static final double TrackWidth = Units.inchesToMeters(23.622); //0.6; //Units.inchesToMeters(23.875);
        // Distance between right and left wheels
        public static final double WheelBase = Units.inchesToMeters(24.212598); //0.615; //Units.inchesToMeters(23.875);
        // Distance between front and back wheels
        public static Translation2d m_frontLeft = new Translation2d(WheelBase / 2.0, TrackWidth / 2.0);
        public static Translation2d m_frontRight = new Translation2d(WheelBase / 2.0, -TrackWidth / 2.0);
        public static Translation2d m_backLeft = new Translation2d(-WheelBase / 2.0, TrackWidth / 2.0);
        public static Translation2d m_backRight = new Translation2d(-WheelBase / 2.0, -TrackWidth / 2.0);
        public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
                m_frontLeft, m_frontRight, m_backLeft, m_backRight);
       
  }

  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kPxController = 1;
    public static final double kPyController = 1;
    public static final double kPThetaController = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond,
          kMaxAngularSpeedRadiansPerSecondSquared
      );
  }
        
  public static class FrontLeftModule {
    public static final int driveID = 14;
    public static final int angleID = 15;
    public static final int encoderID = 3;
    public static final boolean driveMotorReversed = true;
    public static final boolean angleMotorReversed = true;
    public static final boolean absoluteEncoderReversed = false;
    public static final double absoluteEncoderOffset = 0;

  }

  public static class FrontRightModule {
    public static final int driveID = 12;
    public static final int angleID = 17;
    public static final int encoderID = 13;
    public static final boolean driveMotorReversed = true;
    public static final boolean angleMotorReversed = true;
    public static final boolean absoluteEncoderReversed = false;
    public static final double absoluteEncoderOffset = 0;
  }

  public static class BackLeftModule {
    public static final int driveID = 2;
    public static final int angleID = 8;
    public static final int encoderID = 7;
    public static final boolean driveMotorReversed = true;
    public static final boolean angleMotorReversed = true;
    public static final boolean absoluteEncoderReversed = false;
    public static final double absoluteEncoderOffset = 0;
  }

  public static class BackRightModule {
    public static final int driveID = 11;
    public static final int angleID = 10;
    public static final int encoderID = 9;
    public static final boolean driveMotorReversed = true;
    public static final boolean angleMotorReversed = true;
    public static final boolean absoluteEncoderReversed = false;
    public static final double absoluteEncoderOffset = 0;
  }

  public static class OIConstants {
    public static final double deadband = 0.05;
    public static final double limelightdeadband = 0.05;
    }
  }


