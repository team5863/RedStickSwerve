package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeControl extends Command {
    private final Intake m_intake;

    Double speed;

    public IntakeControl(Intake m_intake, Double speed){
        this.m_intake = m_intake;
        this.speed = speed;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {

       m_intake.intake(speed);

    }

    @Override
    public void end(boolean interrupted) {
        m_intake.intake(0);
    }
}
