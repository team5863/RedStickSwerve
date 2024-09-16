package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class Shoot extends SubsystemBase {
    public  CANSparkMax shooterMotor;
    public static PIDController m_PidController;
    public static PIDController h_PidController;
    RelativeEncoder Encoder;

    double
      m_kp = 1,
      m_ki = 0,
      m_kd = 0;

    double
      h_kp = 1,
      h_ki = 0,
      h_kd = 0;

    public Shoot() {

        shooterMotor = new CANSparkMax(Constants.CommandConstants.Shoot.shooterSparkMax, MotorType.kBrushless);
        //armMotor.setSmartCurrentLimit(110, 100);
        Encoder = shooterMotor.getEncoder();

        shooterMotor.restoreFactoryDefaults();

        shooterMotor.setIdleMode(IdleMode.kCoast);

        m_PidController = new PIDController(m_kp, m_ki, m_kd);
        h_PidController = new PIDController(h_kp, h_ki, h_kd);

         m_PidController.setP(m_kp);
         m_PidController.setI(m_ki);
         m_PidController.setD(m_kd);

         h_PidController.setP(h_kp);
         h_PidController.setI(h_ki);
         h_PidController.setD(h_kd);

         m_PidController.setTolerance(1);
    }

    public void shoot(double speed) {
        shooterMotor.set(-speed);
    }

    public void autoShoot(double target) {
        m_PidController.setSetpoint(target);
        double speedOutput = MathUtil.clamp(m_PidController.calculate(Encoder.getPosition()), -0.8, 0.8);
        shoot(-speedOutput);
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Arm Position", Encoder.getPosition());
    }
}
