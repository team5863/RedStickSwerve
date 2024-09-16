package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoot;

public class ShootControl extends Command {
    private final Shoot m_shoot;

    Double speed;

    public ShootControl(Shoot m_shoot, Double speed){
        this.m_shoot = m_shoot;
        this.speed = speed;
        addRequirements(m_shoot);
    }

    @Override
    public void execute() {
        m_shoot.shoot(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shoot.shoot(0);
    }
}
