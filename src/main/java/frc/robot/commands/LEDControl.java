// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

public class LEDControl extends Command {
  LED m_LEDdriver;
  double light;

  public LEDControl(LED m_LEDdriver, double light) {
    addRequirements(m_LEDdriver);

    this.m_LEDdriver = m_LEDdriver;
    this.light = light;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_LEDdriver.set(light);

}

  @Override
  public void end(boolean interrupted) {
    m_LEDdriver.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
