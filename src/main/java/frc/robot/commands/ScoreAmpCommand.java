// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmpCommand extends SequentialCommandGroup {
  /** Creates a new ScoreAmp. */
  public ScoreAmpCommand(DrivetrainSubsystem dts, TelescopeExtendCommand telescopeExtendCommand, TelescopeRetractCommand telescopeRetractCommand) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(dts.goToTargetPos(dts.getTargetPosition(0), 90),
    //             new WaitCommand(0.5),
    //             telescopeExtendCommand,
    //             telescopeRetractCommand);
    addCommands();
  }
}