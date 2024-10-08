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
import frc.robot.commands.AmpAuto;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.Climber1Command;
import frc.robot.commands.Climber2Command;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.LeaveHomeAuto;
import frc.robot.commands.MoveArmToPoseCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.commands.SpeakerCommand;
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
  public static final DriveTrainCommand m_driveTrainCommand = new DriveTrainCommand(m_drivetrainSubsystem);
  public static final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
  public static final ClimberCommand m_climberCommand = new ClimberCommand(m_climberSubsystem);
  public static final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem);
  public static final IntakeOutCommand m_intakeOutCommand = new IntakeOutCommand(m_intakeSubsystem);
  public static final OutakeCommand m_outakeCommand = new OutakeCommand(m_outakeSubsystem);
  public static final TelescopeExtendCommand m_telescopeExtendCommand = new TelescopeExtendCommand(m_telescopeSubsystem);
  public static final TelescopeRetractCommand m_telescopeRetractCommand = new TelescopeRetractCommand(m_telescopeSubsystem);
  public static final MoveArmToPoseCommand m_pickUpPoseCommand = new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_PICKUP_ANGLE);
  public static final MoveArmToPoseCommand m_ampPoseCommand = new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_AMP_ANGLE);
  public static final MoveArmToPoseCommand m_speakerPoseCommand = new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_SHOOTER_ANGLE);
  public static final StartReleasingServoCommand m_startReleasingServoCommand = new StartReleasingServoCommand(m_releaseClimber);
  public static final StopReleasingServoCommand m_stopReleasingServoCommand = new StopReleasingServoCommand(m_releaseClimber);
  public static final SpeakerCommand m_speakerCommand = new SpeakerCommand(m_armSubsystem, m_intakeSubsystem, m_outakeSubsystem);
  public static final LeaveHomeAuto m_leaveHomeAuto = new LeaveHomeAuto(m_drivetrainSubsystem);
  public static final AmpAuto m_ampAuto = new AmpAuto(m_drivetrainSubsystem);
  public static final Climber1Command m_climber1Command = new Climber1Command(m_climberSubsystem);
  public static final Climber2Command m_climber2Command = new Climber2Command(m_climberSubsystem);


  public final AutonomousCommand m_autonomousCommand = new AutonomousCommand(m_drivetrainSubsystem, m_armSubsystem, m_intakeSubsystem, m_outakeSubsystem);



  public RobotContainer() {
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
    Trigger armPickupPoseButton = new JoystickButton(rightJoystick, Constants.GROUND_PICKUP_BUTTON);
    Trigger armAmpPoseButton = new JoystickButton(rightJoystick, Constants.AMP_POSE_BUTTON);
    Trigger intakeOutButton = new JoystickButton(rightJoystick, Constants.INTAKE_OUT_BUTTON);
    Trigger swerveResetButton = new JoystickButton(rightJoystick, Constants.SWERVE_RESET_BUTTON);
    Trigger speakerButton = new JoystickButton(rightJoystick, Constants.SPEAKER_BUTTON);

    // left buttons
    Trigger outakeButton = new JoystickButton(leftJoystick, Constants.OUTTAKE_BUTTON);
    Trigger telescopeExtendButton = new JoystickButton(leftJoystick, Constants.TELESCOPE_EXTEND_BUTTON);
    Trigger telescopeRetractButton = new JoystickButton(leftJoystick, Constants.TELESCOPE_RETRACT_BUTTON);
    Trigger climberButton = new JoystickButton(leftJoystick, Constants.CLIMBER_BUTTON);
    Trigger climber1Button = new JoystickButton(leftJoystick, Constants.CLIMBER1_BUTTON);
    Trigger climber2Button = new JoystickButton(leftJoystick, Constants.CLIMBER2_BUTTON);

    // Trigger armStartButton = new JoystickButton(leftJoystick, Constants.ARM_START_BUTTON);

    intakeButton.whileTrue(m_intakeCommand);
    armPickupPoseButton.onTrue(m_pickUpPoseCommand);
    armAmpPoseButton.onTrue(m_ampPoseCommand);
    intakeOutButton.onTrue(m_intakeOutCommand);
    speakerButton.onTrue(m_speakerCommand);
    swerveResetButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.resetAngle()));

    outakeButton.whileTrue(m_outakeCommand);
    telescopeExtendButton.whileTrue(m_telescopeExtendCommand);
    telescopeRetractButton.whileTrue(m_telescopeRetractCommand);
    climberButton.whileTrue(m_climberCommand);
    climber1Button.whileTrue(m_climber1Command);
    climber2Button.whileTrue(m_climber2Command);

    // armStartButton.whileTrue(m_armCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // An example command will be run in autonomous
    m_autoChooser.addOption("SpeakerAuto", m_autonomousCommand);
    m_autoChooser.addOption("LeaveHomeAuto", m_leaveHomeAuto);

    SmartDashboard.putData("AutoChooser", m_autoChooser);

    return m_autonomousCommand;// Speaker auto
    // return m_leaveHomeAuto;
    // return m_speakerCommand;
  }
}
