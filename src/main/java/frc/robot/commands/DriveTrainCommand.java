// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainCommand extends Command {
  private DrivetrainSubsystem m_dts;
  /** Creates a new DriveTrainCommand. */
  public DrivetrainCommand(DrivetrainSubsystem dts) {
    m_dts = dts;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dts);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = RobotContainer.rightJoystick.getX();
    double y = RobotContainer.rightJoystick.getY();
    double rot = RobotContainer.rightJoystick.getTwist();

    m_dts.setXPowerCommanded(Math.copySign(Math.pow(x, 3), x));
    m_dts.setYPowerCommanded(Math.copySign(Math.pow(y, 3), y));
    m_dts.setRotCommanded(Math.copySign(Math.pow(rot, 3), rot));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
