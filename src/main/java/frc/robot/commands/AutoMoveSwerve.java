// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoMoveSwerve extends Command {
  /** Creates a new AutoMoveSwerve. */
  double initialX;
  double initialY;
  double x;
  double y;
  DrivetrainSubsystem m_drivetrainSubsystem;
  AHRS m_gyro;


  public AutoMoveSwerve(DrivetrainSubsystem drivetrainSubsystem,double x, double y) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    this.x=x;
    this.y=y;
    m_gyro = m_drivetrainSubsystem.m_gyro;
    initialX=m_gyro.getDisplacementX();
    initialY=m_gyro.getDisplacementY();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setXPowerCommanded(0.20);
    m_drivetrainSubsystem.setYPowerCommanded(0.20);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_gyro.getDisplacementX()-initialX>x) {
      m_drivetrainSubsystem.setXPowerCommanded(0);
    }
    if (m_gyro.getDisplacementY()-initialY>y) {
      m_drivetrainSubsystem.setYPowerCommanded(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
