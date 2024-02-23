// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.OutakeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapCommand extends SequentialCommandGroup {
  /** Creates a new TrapCommand. */
  ArmSubsystem m_armSubsystem;
  TelescopeSubsystem m_telescopeSubsystem;
  OutakeSubsystem m_outakeSubsystem;
  public TrapCommand(ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem, OutakeSubsystem outakeSubsystem) {
    m_armSubsystem = armSubsystem;
    m_telescopeSubsystem = telescopeSubsystem;
    m_outakeSubsystem = outakeSubsystem;
    

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveArmToPoseCommand(armSubsystem, 0),
      new WaitCommand(0.5), 
      new TelescopeExtendCommand(telescopeSubsystem), 
      new WaitCommand(3), 
      new InstantCommand(() -> m_outakeSubsystem.setOutakePower()).withTimeout(.1), 
      new WaitCommand(0.5), 
      new InstantCommand(() -> m_outakeSubsystem.zeroOutakePower()).withTimeout(0.1));
  }
}
