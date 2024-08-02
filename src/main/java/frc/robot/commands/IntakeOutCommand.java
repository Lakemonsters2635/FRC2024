// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends Command {
  /** Creates a new IntakeOutCommand. */
  private IntakeSubsystem m_intakeSubsystem;
  private Timer m_timer;
  public IntakeOutCommand(IntakeSubsystem intakeSubsystem) {
    // Set the member variable to parameter

    // Initialize the timer

    // addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset the timer

    // Start the timer

    // Run intake out

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop running intake motor

    // Stop the timer

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Replace with a condition where if the timer's value is higher than 0.2 return true, else return false
    return false;
  }
}
