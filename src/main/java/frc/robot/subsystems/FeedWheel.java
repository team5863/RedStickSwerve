// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
public class FeedWheel extends SubsystemBase {

  Spark feedMotor;
  /** Creates a new FeedWheel. */
  public FeedWheel() {
    feedMotor = new Spark(Constants.CommandConstants.FeedWheel.feedWheelSpark);
  }

  public void feed(double speed) {
    feedMotor.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
