// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.security.cert.TrustAnchor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousCommand. */
  DrivetrainSubsystem m_dts;
  ArmSubsystem m_as;
  IntakeSubsystem m_is;
  OutakeSubsystem m_os;

  public AutonomousCommand(DrivetrainSubsystem dts, ArmSubsystem as, IntakeSubsystem is, OutakeSubsystem os) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_dts = dts;
    m_as = as;
    m_is = is;
    m_os = os;

    addCommands(
      new InstantCommand(()->dts.resetOdometry(new Pose2d(0,0,new Rotation2d()))).withTimeout(0.1),
      new InstantCommand(()->dts.resetAngle()),
      new SpeakerCommand(as, is, os),
      leaveHome(),
      goToSpeaker(),
      new SpeakerCommand(as, is, os)
    );
  }

  public Command goToSpeaker(){
    Pose2d initialPose = m_dts.getPose();

    return new SequentialCommandGroup(m_dts.createPath(
          initialPose,
          new Translation2d(initialPose.getX(),0.9),
          new Pose2d(initialPose.getX(), 1.8, initialPose.getRotation())
      ),
      new InstantCommand(()->m_dts.stopMotors()),
      new IntakeCommand(m_is).withTimeout(0.1));
  }

  public Command leaveHome(){
    Pose2d initialPose = m_dts.getPose();

    return new ParallelRaceGroup(new SequentialCommandGroup(m_dts.createPath(
      initialPose,
      new Translation2d(initialPose.getX(), -0.9),
      new Pose2d(initialPose.getX(), -1.8, initialPose.getRotation())
    ),
    new InstantCommand(()->m_dts.stopMotors())),
    new IntakeCommand(m_is));
  }
}