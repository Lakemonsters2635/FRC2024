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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousCommand. */

  DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public AutonomousCommand(DrivetrainSubsystem dts) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrainSubsystem = dts;

    addCommands(
      new InstantCommand(()->dts.resetOdometry(new Pose2d(0,0,new Rotation2d()))).withTimeout(0.1),
      new InstantCommand(()->dts.resetAngle()),
      goToSpeaker()
    );
  }

  public SequentialCommandGroup goToSpeaker(){
    Pose2d initialPose = m_drivetrainSubsystem.getPose();
    return new SequentialCommandGroup(m_drivetrainSubsystem.createPath(
          initialPose,
          new Translation2d(0.5,1),
          new Pose2d(1, 0, new Rotation2d(initialPose.getRotation().getRadians()))
      ),
      new InstantCommand(()->m_drivetrainSubsystem.stopMotors()));
  }
}