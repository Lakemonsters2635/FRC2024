// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(DrivetrainSubsystem dts, AprilTagChooser aprilTagChooser, NoteTakerCommand noteTakerCommand) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      dts.createPathOnFlight(new Pose2d(2,2, Rotation2d.fromDegrees(180)), 180),
      new WaitCommand(3),
      new ParallelCommandGroup(aprilTagChooser, new InstantCommand(()-> System.out.println("ApriltagChooser: running"))),
      new WaitCommand(3),
      new InstantCommand(()->System.out.println("Create path on flight running")),
      // dts.createPathOnFlight(new Pose2d(dts.getPose().getX(), dts.getPose().getY(), Rotation2d.fromDegrees(0)), 0),
      // new WaitCommand(3),
      new ParallelCommandGroup(noteTakerCommand, new InstantCommand(()->System.out.println("NoteTakerCommand: running"))),
      new WaitCommand(3)
    );
  }
}
