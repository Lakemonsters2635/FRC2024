// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.commands.TelescopeExtendCommand;
import frc.robot.commands.TelescopeRetractCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutakeSubsystem;
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

  //Command 
  public static final DrivetrainCommand m_driveTrainCommand = new DrivetrainCommand(m_drivetrainSubsystem);
  public static final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
  public static final ClimberCommand m_climberCommand = new ClimberCommand(m_climberSubsystem);
  public static final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem);
  public static final OutakeCommand m_outakeCommand = new OutakeCommand(m_outakeSubsystem);
  public static final TelescopeExtendCommand m_telescopeExtendCommand = new TelescopeExtendCommand(m_telescopeSubsystem);
  public static final TelescopeRetractCommand m_telescopeRetractCommand = new TelescopeRetractCommand(m_telescopeSubsystem);

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
    Trigger telescopeExtendButton = new JoystickButton(rightJoystick, Constants.TELESCOPE_EXTEND_BUTTON);
    Trigger telescopeRetractButton = new JoystickButton(rightJoystick, Constants.TELESCOPE_RETRACT_BUTTON);
    Trigger intakeButton = new JoystickButton(rightJoystick, Constants.INTAKE_BUTTON);
    Trigger swerveResetButton = new JoystickButton(rightJoystick, Constants.SWERVE_RESET_BUTTON);
    Trigger modifierButton = new JoystickButton(rightJoystick, Constants.MODIFIER_BUTTON);

    // left buttons
    Trigger outakeButton = new JoystickButton(leftJoystick, Constants.OUTTAKE_BUTTON);
    Trigger armPositionerButton = new JoystickButton(leftJoystick, Constants.ARM_POSITIONER_BUTTON);
    Trigger climberButton = new JoystickButton(leftJoystick, Constants.CLIMBER_BUTTON);
    Trigger trapOuttakeButton = new JoystickButton(leftJoystick, Constants.TRAP_OUTTAKE_BUTTON);
    Trigger armTrapPositionerButton = new JoystickButton(leftJoystick, Constants.ARM_TRAP_POSITIONER_BUTTON);

    intakeButton.whileTrue(m_intakeCommand);
    telescopeExtendButton.whileTrue(m_telescopeExtendCommand);
    telescopeRetractButton.whileTrue(m_telescopeRetractCommand);
    swerveResetButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.resetAngle()));
    modifierButton.onTrue(new InstantCommand(()->m_outakeCommand.speed=Constants.OUTTAKE_SHOOTER_SPEED));
    modifierButton.onFalse(new InstantCommand(()->m_outakeCommand.speed=Constants.OUTTAKE_AMP_SPEED));

    outakeButton.whileTrue(m_outakeCommand);
    climberButton.whileTrue(m_climberCommand);
    trapOuttakeButton.onTrue(new SequentialCommandGroup(
      new InstantCommand(()->m_outakeCommand.speed=Constants.OUTTAKE_TRAP_SPEED),
      m_outakeCommand
    ));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}