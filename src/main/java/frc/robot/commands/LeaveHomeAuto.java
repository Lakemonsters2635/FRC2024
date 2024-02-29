// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveHomeAuto extends SequentialCommandGroup {
  /** Creates a new LeaveHomeAuto. */
  public LeaveHomeAuto(DrivetrainSubsystem dts) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      dts.createPath(
        dts.getPose(),
        new Translation2d(0,-0.9), 
        new Pose2d(0,-1.8, dts.getPose().getRotation()))
    );
  }
}
