// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutakeSubsystem;

public class OutakeCommand extends Command {
  /** Creates a new IntakeOutCommand. */
  private OutakeSubsystem m_outakeSubsystem;
  public OutakeCommand(OutakeSubsystem outakeSubsystem) {

    m_outakeSubsystem = outakeSubsystem;
    addRequirements(m_outakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_outakeSubsystem.setOutakePower();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_outakeSubsystem.zeroOutakePower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
