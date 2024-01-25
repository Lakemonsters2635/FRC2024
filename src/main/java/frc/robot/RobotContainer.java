// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveTrainCommand;
import edu.wpi.first.wpilibj.Joystick;
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
  public static ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  //Command 
  public static final DriveTrainCommand m_driveTrainCommand = new DriveTrainCommand(m_drivetrainSubsystem);
  public final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
  public static ClimberCommand m_climberCommand = new ClimberCommand(m_climberSubsystem);

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

    Trigger ArmStartButton = new JoystickButton(leftJoystick, Constants.ARM_START_BUTTON);
    Trigger climberButton = new JoystickButton(leftJoystick, Constants.CLIMBER_BUTTON);
    Trigger swerveResetButton = new JoystickButton(rightJoystick, 1);
    Trigger intakeButton = new JoystickButton(leftJoystick, Constants.INTAKE_BUTTON);

    ArmStartButton.whileTrue(m_armCommand);
    climberButton.onTrue(m_climberCommand);
    swerveResetButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.resetAngle()) );
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