// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeExtendCommand extends Command {
  public TelescopeSubsystem m_telescopeSubsystem;
  /** Creates a new TelescopeExtendComman. */

  public TelescopeExtendCommand(TelescopeSubsystem telescopeSubsystem) {
    m_telescopeSubsystem = telescopeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_telescopeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_telescopeSubsystem.extendTelescope();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_telescopeSubsystem.stopTelescope();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_telescopeSubsystem.telescopeEncoder.get()>500) {
      return true;
    }
    return false;
  }
}
