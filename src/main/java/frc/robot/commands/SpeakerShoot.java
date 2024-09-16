package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Serialize;
import frc.robot.subsystems.Shoot;

public class SpeakerShoot extends Command {
    private final Shoot m_shoot;
    private final Serialize m_serialize;

    Double speed;
    Double speed2;

    public SpeakerShoot(Shoot m_shoot, Double speed, Serialize m_serialize, Double speed2){
        this.m_shoot = m_shoot;
        this.m_serialize = m_serialize;
        this.speed = speed;
        this.speed2 = speed2;
        addRequirements(m_shoot);
    }

    @Override
    public void execute() {
     
        m_shoot.shoot(speed);
        m_serialize.serialize(speed2);
        
    }

    @Override
    public void end(boolean interrupted) {
        m_shoot.shoot(0);
        m_serialize.serialize(0);
    }
}
