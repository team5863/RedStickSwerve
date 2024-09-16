package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.CommandConstants.Intake.intakeSparkMax, MotorType.kBrushless);
    }

    public void intake(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {

    }
}
