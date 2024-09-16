// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autocommands.autoIntake;
import frc.robot.autocommands.autoSpeakerShoot;
import frc.robot.autocommands.limeSpeakerShoot;
import frc.robot.commands.FeedControl;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.LEDControl;
import frc.robot.commands.LimeAim;
import frc.robot.commands.SpeakerShoot;
//import frc.robot.autos.SwerveAuto;
import frc.robot.commands.SwerveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Serialize;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.FeedWheel;
//import frc.robot.subsystems.ColorSensor;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  // The robot's subsystems and commands are defined here...
    public static Serialize m_serialize = new Serialize();
    public static Shoot m_shoot = new Shoot();
    public static Intake m_intake = new Intake();
    public static LED m_LEDcontrol = new LED();
    public static FeedWheel m_feed = new FeedWheel();

    //public static ColorSensor m_colorSensor = new ColorSensor();

  public static SwerveDriveTrain m_driveTrain = new SwerveDriveTrain();
  //private final Field2d field;

  public static LimeAim m_limeDrive = new LimeAim(m_driveTrain, () -> 0.0, () -> 0.0, () -> 0.0, () -> false);

  private final Joystick driver = new Joystick(0);
  private final int drivetrainSpeedX = XboxController.Axis.kLeftY.value;
  private final int drivetrainSpeedY = XboxController.Axis.kLeftX.value;
  private final int drivetrainRotation = XboxController.Axis.kRightX.value;
  private final int fieldRelativeButton = XboxController.Button.kLeftBumper.value;

  private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kA.value);
  //private final JoystickButton outtakeButtonDriver = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton LimeAimButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton resetButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  private final JoystickButton setLEDredButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton setLEDblueButton = new JoystickButton(driver, XboxController.Button.kX.value);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  public final Joystick operator = new Joystick(1);

      private final JoystickButton sourceIntakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
      private final JoystickButton outtakeButtonOperator = new JoystickButton(operator, XboxController.Button.kA.value);
      private final JoystickButton ampShootButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
      private final JoystickButton speakerShootButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
      private final JoystickButton climbLowerButton = new JoystickButton(operator, XboxController.Button.kX.value);
      private final JoystickButton climbRaiseButton = new JoystickButton(operator, XboxController.Button.kY.value);
    

  //auto
    private final SendableChooser<Command> autoChooser;

  //List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("TestAuto");
  //Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("4NoteBlueAuto");
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

      m_driveTrain.setDefaultCommand(
        new SwerveCommand(
          m_driveTrain,
          () -> driver.getRawAxis(drivetrainSpeedX),
          () -> driver.getRawAxis(drivetrainSpeedY),
          () -> driver.getRawAxis(drivetrainRotation),
          () -> !driver.getRawButton(fieldRelativeButton)
        )
      );

      /*field = new Field2d();
      SmartDashboard.putData("field", field);

      PathPlannerLogging.setLogCurrentPoseCallback() ->{
        field.setRobotPose();
      };*/


    //autos
    
    //autoChooser.setDefaultOption("SwerveAuto", new SwerveAuto(m_driveTrain))
    //String currentAuto = autoChooser.getSelected().getName();
    //Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(currentAuto);
    
    //NamedCommands.registerCommand("set Heading", Commands.run(() -> m_driveTrain.setHeading(0)));
    //NamedCommands.registerCommand("set Heading", Commands.run(() -> m_driveTrain.setHeading(startingPose.getRotation().getDegrees())));
    // NamedCommands.registerCommand("print hello", Commands.print("hello"));


    NamedCommands.registerCommand("SpeakerShoot", new autoSpeakerShoot(m_shoot, m_serialize, m_feed));
    NamedCommands.registerCommand("Intake", new autoIntake(m_intake, m_feed));
    NamedCommands.registerCommand("LimeSpeakerShoot", new limeSpeakerShoot(m_shoot, m_serialize, m_feed, m_driveTrain, m_limeDrive));
     //NamedCommands.registerCommand("Set Gyro", m_driveTrain.setHeading);


     autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
   // SmartDashboard.putString("Current Auto",autoChooser.getSelected().getName());
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    resetButton.whileTrue(new InstantCommand(() -> m_driveTrain.zeroHeading(), m_driveTrain));


    intakeButton.whileTrue(new ParallelDeadlineGroup(
      new WaitCommand(0.01))
      .andThen(
      new ParallelCommandGroup(
        new IntakeControl(m_intake, -0.3),
        new FeedControl(m_feed, 0.35))));

    //outtakeButtonDriver.whileTrue(new IntakeControl(m_intake, 0.3));
   
    LimeAimButton.whileTrue(
      new LimeAim(m_driveTrain,
          () -> driver.getRawAxis(drivetrainSpeedX),
          () -> driver.getRawAxis(drivetrainSpeedY),
          () -> driver.getRawAxis(drivetrainRotation),
          () -> !driver.getRawButton(fieldRelativeButton)
        )
    );
    setLEDblueButton.whileTrue(new LEDControl(m_LEDcontrol, 0.87));
    setLEDredButton.whileTrue(new LEDControl(m_LEDcontrol, 0.61));

    

    //Operator
    sourceIntakeButton.whileTrue(new SpeakerShoot(m_shoot, 0.15, m_serialize, 0.15));

    /*outtakeButtonOperator.whileTrue(new ParallelCommandGroup(
        new IntakeControl(m_intake, 0.3),
        new FeedControl(m_feed, -0.3)));*/

      outtakeButtonOperator.whileTrue(new ParallelDeadlineGroup(
        new WaitCommand(0.01))
      .andThen(
      new ParallelCommandGroup(
        new ParallelCommandGroup(
      new IntakeControl(m_intake, 0.3),
      new FeedControl(m_feed, -0.35)))));

    ampShootButton.whileTrue(new ParallelDeadlineGroup(
      new WaitCommand(0.01))
      .andThen(
      new ParallelCommandGroup(
        new LEDControl(m_LEDcontrol, 0.77),
        new SpeakerShoot(m_shoot, -0.09, m_serialize, -0.15),
        new FeedControl(m_feed, 0.2))));

    speakerShootButton.whileTrue(new ParallelDeadlineGroup(
      new WaitCommand(0.3),
      new SpeakerShoot(m_shoot, -1.0, m_serialize, 0.0))
      .andThen(
      new ParallelCommandGroup(
        new LEDControl(m_LEDcontrol, 0.77),
        new SpeakerShoot(m_shoot, -1.0, m_serialize, -0.9),
        new FeedControl(m_feed, 0.3))));

    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
     
  }
}