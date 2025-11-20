// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AmpOutakeCommand;
import frc.robot.commands.AmpSequenceCommand;
import frc.robot.commands.ArmThrottleCommand;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.FarSpeakerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.MoveArmToPoseCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.commands.OuttakeInCommand;
import frc.robot.commands.SpeakerCommand;
import frc.robot.commands.SpeakerDriverCommand;
import frc.robot.commands.TrapShootCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import frc.robot.subsystems.OutakeSubsystem;

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
  public static final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final OutakeSubsystem m_outakeSubsystem = new OutakeSubsystem();
  public static final ObjectTrackerSubsystem m_objectTrackerSubsystem = new ObjectTrackerSubsystem("Eclipse");
  public static final ObjectTrackerSubsystem m_objectTrackerSubsystemFPS = new ObjectTrackerSubsystem("fps");
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static final ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();

  //Command 
  public static final DriveTrainCommand m_driveTrainCommand = new DriveTrainCommand(m_drivetrainSubsystem);
  public static final ArmThrottleCommand m_armThrottleCommand = new ArmThrottleCommand(m_armSubsystem);
  public static final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem);
  public static final IntakeOutCommand m_intakeOutCommand = new IntakeOutCommand(m_intakeSubsystem);
  public static final OutakeCommand m_outakeCommand = new OutakeCommand(m_outakeSubsystem);
  public static final AmpOutakeCommand m_ampOutakeCommand = new AmpOutakeCommand(m_outakeSubsystem);
  public static final MoveArmToPoseCommand m_pickUpPoseCommand = new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_PICKUP_ANGLE);
  public static final MoveArmToPoseCommand m_ampPoseCommand = new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_AMP_ANGLE);
  public static final MoveArmToPoseCommand m_speakerPoseCommand = new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_SHOOTER_ANGLE);
  public static final SpeakerCommand m_speakerCommand = new SpeakerCommand(m_armSubsystem, m_intakeSubsystem, m_outakeSubsystem);
  public static final TrapShootCommand m_TrapShootCommand = new TrapShootCommand(m_armSubsystem, m_intakeSubsystem, m_outakeSubsystem);
  public static final TrapShootCommand m_trapShootCommand = new TrapShootCommand(m_armSubsystem, m_intakeSubsystem, m_outakeSubsystem);
  public static final AmpSequenceCommand m_ampSequenceCommand = new AmpSequenceCommand(m_armSubsystem, m_intakeSubsystem, m_outakeSubsystem);
  public static final OuttakeInCommand m_outtakeInCommand = new OuttakeInCommand(m_outakeSubsystem);
  public static final MoveArmToPoseCommand m_moveArmToPoseSpeaker = new MoveArmToPoseCommand(m_armSubsystem, 54);
  public static final MoveArmToPoseCommand m_moveArmBalance = new MoveArmToPoseCommand(m_armSubsystem, (int)Constants.ARM_ENCODER_OFFSET*360);// TODO
  public static final ParallelCommandGroup m_armAmpPoseCommand = new ParallelCommandGroup(m_ampPoseCommand, m_intakeOutCommand);
  // public static final SetRobotRot m_setRobotRot90 = new SetRobotRot(m_drivetrainSubsystem, 90);


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
    Trigger swerveResetButton = new JoystickButton(rightJoystick, Constants.SWERVE_RESET_BUTTON);
    Trigger speakerButton = new JoystickButton(rightJoystick, Constants.SPEAKER_BUTTON);
    Trigger trapShootButton = new JoystickButton(rightJoystick, Constants.TRAP_SHOOT_BUTTON);
    Trigger outtakeInButton = new JoystickButton(rightJoystick, Constants.OUTTAKE_IN_BUTTON);
    Trigger armBalanceButton = new JoystickButton(rightJoystick, Constants.BALANCE_BUTTON);
    Trigger resetOdometryButton = new JoystickButton(rightJoystick, 11);

    Trigger viewAprilTagButton = new JoystickButton(rightJoystick, 9);
    
    // Trigger testButton = new JoystickButton(rightJoystick, 10);

    // left buttons
    Trigger outakeButton = new JoystickButton(leftJoystick, Constants.OUTTAKE_BUTTON);
    Trigger farSpeakerButton = new JoystickButton(leftJoystick, Constants.FAR_SHOOTER_BUTTON);
    Trigger visionCombinedTrap = new JoystickButton(leftJoystick, 7);
    Trigger visionMovement = new JoystickButton(leftJoystick, 12);
    Trigger toggleLeftOutput = new JoystickButton(leftJoystick, 5);
    Trigger toggleRightOutput = new JoystickButton(leftJoystick, 6);
    // Trigger armStartButton = new JoystickButton(leftJoystick, Constants.ARM_START_BUTTON);

    intakeButton.whileTrue(m_intakeCommand);
    armPickupPoseButton.onTrue(m_pickUpPoseCommand);
    // armAmpPoseButton.onTrue(new ParallelCommandGroup(m_ampPoseCommand, m_intakeOutCommand));
    armAmpPoseButton.onTrue(m_armAmpPoseCommand);
    // intakeOutButton.onTrue(m_intakeOutCommand);
    speakerButton.onTrue(new SpeakerDriverCommand(m_armSubsystem, m_intakeSubsystem, m_outakeSubsystem));
    swerveResetButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.resetAngle()));
    outtakeInButton.whileTrue(m_outtakeInCommand);
    trapShootButton.onTrue(m_trapShootCommand);
    armBalanceButton.onTrue(m_armAmpPoseCommand);
    toggleLeftOutput.onTrue(new InstantCommand(()->m_arduinoSubsystem.toggleLeftEnableOut()));
    toggleRightOutput.onTrue(new InstantCommand(()->m_arduinoSubsystem.toggleRightEnableOut()));
    resetOdometryButton.onTrue(
      
      new SequentialCommandGroup(
        new InstantCommand(()->m_drivetrainSubsystem.resetAngle()),

        new InstantCommand(()->m_drivetrainSubsystem.zeroOdometry())
      )
    );
    viewAprilTagButton.onTrue(new SequentialCommandGroup(
      new InstantCommand(()->m_objectTrackerSubsystem.data()),
      new InstantCommand(()->SmartDashboard.putString("ClosestObjectVision", m_objectTrackerSubsystem.getClosestAprilTag().toString())),
      new InstantCommand(()->SmartDashboard.putString("VisionObjectsJson",m_objectTrackerSubsystem.getObjectsJson()))
    ));
    // testButton.onTrue(m_moveArmToPoseSpeaker);

    outakeButton.whileTrue(m_ampOutakeCommand);
    farSpeakerButton.onTrue(new FarSpeakerCommand(m_armSubsystem, m_intakeSubsystem, m_outakeSubsystem));
    // climberUpButton.whileTrue(m_climberUpCommand);
    // climber1UpButton.whileTrue(m_climber1UpCommand);
    // climber2UpButton.whileTrue(m_climber2UpCommand);
    // climberDownButton.whileTrue(m_climberDownCommand);
    // climber1DownButton.whileTrue(m_climber1DownCommand);
    // climber2DownButton.whileTrue(m_climber2DownCommand);
    // setRobotRotationButton.whileTrue(m_setRobotRot90);
    // setRobotRotationButton2.whileTrue(m_setRobotArm);

  
    // armStartButton.whileTrue(m_armThrottleCommand);
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

    // An example command will be run in autonomous
   
    //m_autoChooser.addOption("escapeLeft", m_autonomousCommands.escapeLeft());  DOESNT WORKz
    // m_autoChooser.addOption("LeaveHomeAuto", m_leaveHomeAuto);
    // m_autoChooser.addOption("AmpAuto", m_ampAuto);


    SmartDashboard.putData("AutoChooser", m_autoChooser);
    SmartDashboard.putData("AlianceChooser", m_alianceChooser);

    return m_autoChooser;
  }
}
