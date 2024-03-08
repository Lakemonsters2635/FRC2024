// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveHomeAuto extends SequentialCommandGroup {
  /** Creates a new LeaveHomeAuto. */
  public LeaveHomeAuto(DrivetrainSubsystem dts) {
    boolean asdf = true;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->dts.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
      new InstantCommand(()->dts.resetAngle()),
      // dts.createPath(
      //   new Pose2d(0,0,new Rotation2d(0)),
      //   new Translation2d(0,-0.1), 
      //   new Pose2d(0,-2.1, new Rotation2d(0))
      // ) //was short at y =-1.8     1 unit=44in  1ft= 0.2732 units

      dts.createPath(
        new Pose2d(0,0,new Rotation2d(Math.toRadians(-90))),
        new Translation2d(0,-0.5), 
        new Pose2d(0,-1.0, new Rotation2d(Math.toRadians(-90)))
      ), 

      dts.createPath(
        new Pose2d(0,-1,new Rotation2d(Math.toRadians(90))),
        new Translation2d(0,-0.5), 
        new Pose2d(0,0, new Rotation2d(Math.toRadians(90)))
      ) 
    );
  }
}
