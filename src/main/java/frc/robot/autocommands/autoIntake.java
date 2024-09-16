package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FeedControl;
import frc.robot.commands.IntakeControl;
import frc.robot.subsystems.*;

public class autoIntake extends SequentialCommandGroup{

    public autoIntake(Intake m_intake, FeedWheel m_feedwheel){
        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(1),
                    new IntakeControl(m_intake, -0.3),
                    new FeedControl(m_feedwheel, 0.5)
                    )
                );
    }
    
}
