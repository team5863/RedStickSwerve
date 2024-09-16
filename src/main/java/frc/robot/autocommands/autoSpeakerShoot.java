package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FeedControl;
import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.*;

public class autoSpeakerShoot extends SequentialCommandGroup{

    public autoSpeakerShoot(Shoot m_shooter, Serialize m_serialize, FeedWheel m_feedwheel){
        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(0.6),
                    new SpeakerShoot(m_shooter, -1.0, m_serialize, -0.9),
                    new FeedControl(m_feedwheel, 0.3)
                    )
                );
    }
    
}
