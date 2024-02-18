// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPoseCommand extends Command {
  /** Creates a new MoveArmToPoseCommand. */
  private ArmSubsystem m_armSubsystem;
  private int m_angle;

  public MoveArmToPoseCommand(ArmSubsystem armSubsystem, int angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_angle = angle;
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setPosTarget(m_angle);
    System.out.println("MOVE ARM COMMANDED");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
