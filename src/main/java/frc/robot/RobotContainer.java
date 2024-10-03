// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

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
  public static final ObjectTrackerSubsystem m_objectTrackerSubsystem = new ObjectTrackerSubsystem("Eclipse");
 
  //Command 
  public static final DrivetrainCommand m_driveTrainCommand = new DrivetrainCommand(m_drivetrainSubsystem);
  public static final AutonomousCommands m_autonomousCommands = new AutonomousCommands(m_drivetrainSubsystem);


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
    Trigger swerveResetButton = new JoystickButton(rightJoystick, Constants.SWERVE_RESET_BUTTON);
    Trigger resetOdometryButton = new JoystickButton(rightJoystick, 11);

    // left buttons

    swerveResetButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.resetAngle()));
    resetOdometryButton.onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->m_drivetrainSubsystem.resetAngle()),
        new InstantCommand(()->m_drivetrainSubsystem.zeroOdometry())
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SendableChooser<Command> getAutonomousCommand() {
    SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    SendableChooser<Command> m_alianceChooser = new SendableChooser<>();

    // m_alianceChooser.addOption("red", new InstantCommand(()->m_drivetrainSubsystem.selectAliance("red")));
    // m_alianceChooser.addOption("blue", new InstantCommand(()->m_drivetrainSubsystem.selectAliance("blue")));
    // m_alianceChooser.addOption("FMS", new InstantCommand(()->m_drivetrainSubsystem.selectAliance("FMS")));

    SmartDashboard.putData("AutoChooser", m_autoChooser);
    SmartDashboard.putData("AlianceChooser", m_alianceChooser);

    return m_autoChooser;
  }
}
