package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FeedControl;
import frc.robot.commands.LimeAim;
import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.*;

public class limeSpeakerShoot extends SequentialCommandGroup{

    public limeSpeakerShoot(Shoot m_shooter, Serialize m_serialize, FeedWheel m_feedwheel, SwerveDriveTrain m_driveTrain, LimeAim m_limeAim){
        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(2),
                new LimeAim(m_driveTrain,
                    () -> 0.0,
                    () -> 0.0,
                    () -> 0.0,
                    () -> false
                )
            ),

                new ParallelDeadlineGroup(
                    new WaitCommand(0.3),
                    new SpeakerShoot(m_shooter, -1.0, m_serialize, -0.9),
                    new FeedControl(m_feedwheel, 0.3)
                )
                   
            );
    }
    
}
