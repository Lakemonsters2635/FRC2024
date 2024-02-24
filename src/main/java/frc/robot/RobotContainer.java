// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AprilTagChooser;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.MoveArmToPoseCommand;
import frc.robot.commands.NoteTakerCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.commands.ScoreAmpCommand;
import frc.robot.commands.ScoreShooterCommand;
import frc.robot.commands.StartReleasingServoCommand;
import frc.robot.commands.StopReleasingServoCommand;
import frc.robot.commands.TelescopeExtendCommand;
import frc.robot.commands.TelescopeRetractCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutakeSubsystem;
import frc.robot.subsystems.ReleaseClimber;
import frc.robot.subsystems.TelescopeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Joysticks
  public static final Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public static final Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  // Subsystems
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final OutakeSubsystem m_outakeSubsystem = new OutakeSubsystem();
  public static final TelescopeSubsystem m_telescopeSubsystem = new TelescopeSubsystem();
  public static final ReleaseClimber m_releaseClimber = new ReleaseClimber();
  // public final ObjectTrackerSubsystem m_objectTrackerSubsystemNoteCam = new ObjectTrackerSubsystem("NoteCam");
  // public final ObjectTrackerSubsystem m_objectTrackerSubsystemAprilTagPro = new ObjectTrackerSubsystem("AprilTagPro");

  //Command 
  public static final DrivetrainCommand m_driveTrainCommand = new DrivetrainCommand(m_drivetrainSubsystem);
  public static final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
  public static final ClimberCommand m_climberCommand = new ClimberCommand(m_climberSubsystem);
  public static final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem);
  public static final IntakeOutCommand m_intakeOutCommand = new IntakeOutCommand(m_intakeSubsystem);
  public static final OutakeCommand m_outakeCommand = new OutakeCommand(m_outakeSubsystem);
  public static final TelescopeExtendCommand m_telescopeExtendCommand = new TelescopeExtendCommand(m_telescopeSubsystem);
  public static final TelescopeRetractCommand m_telescopeRetractCommand = new TelescopeRetractCommand(m_telescopeSubsystem);
  public static final MoveArmToPoseCommand m_armPickUpPoseCommand = new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_PICKUP_ANGLE);
  public static final MoveArmToPoseCommand m_armAmpPoseCommand = new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_AMP_ANGLE);
  public static final MoveArmToPoseCommand m_outtakePoseCommand = new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_OUTTAKE_ANGLE);
  public static final StartReleasingServoCommand m_startReleasingServoCommand = new StartReleasingServoCommand(m_releaseClimber);
  public static final StopReleasingServoCommand m_stopReleasingServoCommand = new StopReleasingServoCommand(m_releaseClimber);
  private final ScoreAmpCommand m_scoreAmpCommand = new ScoreAmpCommand(m_drivetrainSubsystem, m_telescopeExtendCommand, m_telescopeRetractCommand);
  private final ScoreShooterCommand m_scoreShooterCommand = new ScoreShooterCommand(m_drivetrainSubsystem, m_telescopeExtendCommand, m_telescopeRetractCommand);
  private final AprilTagChooser m_aprilTagChooser = new AprilTagChooser();
  private final NoteTakerCommand m_noteTakerCommand = new NoteTakerCommand(m_drivetrainSubsystem, m_telescopeExtendCommand, m_telescopeRetractCommand);
  public final AutonomousCommand m_autonomousCommand; //= new AutonomousCommand(m_drivetrainSubsystem);


  public RobotContainer() {
    m_autonomousCommand = new AutonomousCommand(m_drivetrainSubsystem);
    SmartDashboard.putString("AutonomousCommand", "empty");

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
    //creating buttons

    // right buttons
    Trigger intakeButton = new JoystickButton(rightJoystick, Constants.INTAKE_BUTTON);
    Trigger intakeOutButton = new JoystickButton(rightJoystick, Constants.INTAKE_OUT_BUTTON);
    Trigger outakeButton = new JoystickButton(rightJoystick, Constants.OUTTAKE_BUTTON);
    Trigger swerveResetButton = new JoystickButton(rightJoystick, Constants.SWERVE_RESET_BUTTON);
    Trigger armAmpPoseButton = new JoystickButton(rightJoystick, Constants.MOVE_ARM_TO_AMP_BUTTON);
    Trigger armPickupPoseButton = new JoystickButton(rightJoystick, Constants.GROUND_PICKUP_BUTTON);
    Trigger armOuttakePoseButton = new JoystickButton(rightJoystick, Constants.OUTTAKE_POSE_BUTTON);
    Trigger startReleasingServoButton = new JoystickButton(rightJoystick, Constants.START_RELEASING_SERVO_BUTTON);
    Trigger stopReleasingServoButton = new JoystickButton(rightJoystick, Constants.STOP_RELEASING_SERVO_BUTTON);

    // left buttons
    Trigger telescopeExtendButton = new JoystickButton(leftJoystick, Constants.TELESCOPE_EXTEND_BUTTON);
    Trigger telescopeRetractButton = new JoystickButton(leftJoystick, Constants.TELESCOPE_RETRACT_BUTTON);
    Trigger armStartButton = new JoystickButton(leftJoystick, Constants.ARM_START_BUTTON);
    Trigger climberButton = new JoystickButton(leftJoystick, Constants.CLIMBER_BUTTON);

    intakeButton.whileTrue(m_intakeCommand);
    intakeOutButton.whileTrue(m_intakeOutCommand);
    outakeButton.whileTrue(m_outakeCommand);
    swerveResetButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.resetAngle()));
    armAmpPoseButton.onTrue(m_armAmpPoseCommand);
    armPickupPoseButton.onTrue(m_armPickUpPoseCommand);
    armOuttakePoseButton.onTrue(m_outtakePoseCommand);
    startReleasingServoButton.onTrue(m_startReleasingServoCommand);
    stopReleasingServoButton.onTrue(m_stopReleasingServoCommand);
    
    telescopeExtendButton.whileTrue(m_telescopeExtendCommand);
    telescopeRetractButton.whileTrue(m_telescopeRetractCommand);
    armStartButton.whileTrue(m_armCommand);
    climberButton.whileTrue(m_climberCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    m_autoChooser.addOption("Test Auto", m_autonomousCommand);

    SmartDashboard.putData("AutoChooser", m_autoChooser);

    System.out.println("getAutonomousCommand is runned");
    return m_autonomousCommand;
    // return m_autoChooser.getSelected();
  }
}
