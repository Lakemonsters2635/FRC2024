// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPoseCommand extends Command {
  /** Creates a new MoveArmToPoseCommand. */
  ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public int angle;
  public MoveArmToPoseCommand(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
  }

  public MoveArmToPoseCommand(ArmSubsystem armSubsystem, int angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (angle == 0) {
    m_armSubsystem.setPosTarget(70);
    } else {
      m_armSubsystem.setPosTarget(angle);
    }
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
