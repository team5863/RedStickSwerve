// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

  public class ColorSensor extends SubsystemBase {
    
      public ColorSensor()  {
        ColorSensor colorSensor = new ColorSensor();
      }
      
      public static int getProximity()  {

        return ColorSensor.getProximity();
      }

  @Override
  public void periodic()  {

    SmartDashboard.putNumber("Proximity:", ColorSensor.getProximity());
    
  }
}
