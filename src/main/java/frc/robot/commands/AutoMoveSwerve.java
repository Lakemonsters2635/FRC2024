// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoMoveSwerve extends Command {
  /** Creates a new AutoMoveSwerve. */
  Timer timer = new Timer();
  double xTime;
  double yTime;
  double x;
  double y;
  DrivetrainSubsystem m_drivetrainSubsystem;
  public AutoMoveSwerve(DrivetrainSubsystem drivetrainSubsystem,double x, double y) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    this.x=x;
    this.y=y;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    m_drivetrainSubsystem.setXPowerCommanded(0.20);
    m_drivetrainSubsystem.setYPowerCommanded(0.20);
    xTime = x/Constants.CHANGE_IN_X_PER_SECOND;
    yTime = y/Constants.CHANGE_IN_Y_PER_SECOND;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get()>xTime) {
      m_drivetrainSubsystem.setXPowerCommanded(0);
    }
    if (timer.get()>yTime) {
      m_drivetrainSubsystem.setYPowerCommanded(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get()>Math.max(xTime, yTime)) {
      return true;
    }
    return false;
  }
}
