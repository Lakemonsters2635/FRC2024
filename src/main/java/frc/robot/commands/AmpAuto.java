// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpAuto extends SequentialCommandGroup {
  /** Creates a new AmpAuto. */
  DrivetrainSubsystem m_dts;

  public AmpAuto(DrivetrainSubsystem dts) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_dts = dts;
    addCommands(
      new InstantCommand(()->dts.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
      new InstantCommand(()->dts.resetAngle()),
      goToAmp()
    );
  }

  public Command goToAmp(){
    // TODO: get the reference angle to start and then apply your rotation with respect to that 
    //       baseline rotation angle.
    return m_dts.createPath(
      new Pose2d(0,0, Rotation2d.fromDegrees(0)), 
      new Translation2d(0, 0.63), 
      new Pose2d(0,1.26, Rotation2d.fromDegrees(90))
    );
  }
}
