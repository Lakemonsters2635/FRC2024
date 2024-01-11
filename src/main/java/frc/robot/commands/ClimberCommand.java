// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  ClimberSubsystem m_climberSubsystem;


  public ClimberCommand(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_climberSubsystem = climberSubsystem;
  addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberSubsystem.runClimber();
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_climberSubsystem.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
