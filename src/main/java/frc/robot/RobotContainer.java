// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.commands.AprilTagChooser;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.NoteTakerCommand;
import frc.robot.commands.ScoreAmpCommand;
import frc.robot.commands.ScoreShooterCommand;
import frc.robot.commands.TelescopeExtendCommand;
import frc.robot.commands.TelescopeRetractCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  public final ArmSubsystem m_armSubsystem = new ArmSubsystem(); 
  public final TelescopeSubsystem m_telescopeSubsystem = new TelescopeSubsystem();

  //Command 
  private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
  private final TelescopeExtendCommand m_telescopeExtendCommand = new TelescopeExtendCommand(m_telescopeSubsystem);
  private final TelescopeRetractCommand m_telescopeRetractCommand = new TelescopeRetractCommand(m_telescopeSubsystem);
  private final ScoreAmpCommand m_scoreAmpCommand = new ScoreAmpCommand(m_drivetrainSubsystem, m_telescopeExtendCommand, m_telescopeRetractCommand);
  private final ScoreShooterCommand m_scoreShooterCommand = new ScoreShooterCommand(m_drivetrainSubsystem, m_telescopeExtendCommand, m_telescopeRetractCommand);
  private final AprilTagChooser m_aprilTagChooser = new AprilTagChooser();
  private final NoteTakerCommand m_noteTakerCommand = new NoteTakerCommand(m_drivetrainSubsystem, m_telescopeExtendCommand, m_telescopeRetractCommand);
  private final AutonomousCommand m_autonomousCommand = new AutonomousCommand(m_drivetrainSubsystem, m_aprilTagChooser, m_noteTakerCommand);

  private SendableChooser<Command> m_autoChooser;

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

    Trigger armStartButton = new JoystickButton(leftJoystick, Constants.ARM_START_BUTTON);
    Trigger noteTakerButton = new JoystickButton(leftJoystick, Constants.NOTE_TAKER_BUTTON);
    Trigger swerveResetButton = new JoystickButton(rightJoystick, Constants.SWERVE_RESET_BUTTON);
    Trigger aprilTagButton = new JoystickButton(rightJoystick, Constants.APRIL_TAG_BUTTON);

    armStartButton.whileTrue(m_armCommand);
    swerveResetButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.resetAngle()));
    aprilTagButton.onTrue(m_aprilTagChooser.choose());
    noteTakerButton.onTrue(m_noteTakerCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SendableChooser<Command> getAutonomousCommand() {
    // An example command will be run in autonomous
    m_autoChooser = new SendableChooser<>();
    m_autoChooser.addOption("Autonomous", m_autonomousCommand);

    SmartDashboard.putData("AutoChooser", m_autoChooser);

    return m_autoChooser;
  }
}