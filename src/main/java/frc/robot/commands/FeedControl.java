// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeedWheel;

public class FeedControl extends Command {
  private final FeedWheel m_feed;
  Double speed; 
  /** Creates a new FeedControl. */
  
  public FeedControl(FeedWheel m_feed, Double speed ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_feed = m_feed;
    this.speed = speed;
    addRequirements(m_feed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feed.feed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feed.feed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
