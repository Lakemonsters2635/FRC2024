// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapShootCommand extends SequentialCommandGroup {
  ArmSubsystem m_armSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  OutakeSubsystem m_outakeSubsystem;
  /** Creates a new TrapShootCommand. */
  public TrapShootCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, OutakeSubsystem outakeSubsystem) {
    m_armSubsystem = armSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_outakeSubsystem = outakeSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup( new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_TRAP_SHOOT_ANGLE),
                                          new SequentialCommandGroup( new IntakeOutCommand(m_intakeSubsystem),
                                                                      new WaitCommand(0.2),
                                                                      new InstantCommand(()->m_outakeSubsystem.setOutakePower()).withTimeout(0.1)
                                                                    )
                                        ), 
                new WaitCommand(0.6),
                new InstantCommand(()->m_intakeSubsystem.inIntake()).withTimeout(0.1),
                new WaitCommand(0.3),
                new InstantCommand(()->m_intakeSubsystem.stopIntake()).withTimeout(0.1),
                new InstantCommand(()->m_outakeSubsystem.zeroOutakePower()),
                new MoveArmToPoseCommand(m_armSubsystem, Constants.ARM_PICKUP_ANGLE).withTimeout(0.3));
  }
}
