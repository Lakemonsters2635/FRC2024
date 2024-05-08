// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetRobotRotArm extends Command {
  /** Creates a new SetRobotRot. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  PIDController pid;
  double initialRotation;
  double rotaionSpeed;
  double targetRotation;
  double yPowerCommanded;
  double xPowerCommanded;
  Joystick rightJoystick;
  Joystick leftJoystick;
  private double fbMotorPower;
  public SetRobotRotArm(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem =drivetrainSubsystem;
    // pid = new PIDController(0.09504, 0, 0.00528);// original PD
    // Higher kp makes it oscillate more as you drive because our robot naturally turns as it drives
    pid = new PIDController(0.05, 0, 0.005);// working PD
    // pid = new PIDController(0.079, 0.396, 0.00396);// Classic PID
    // pid = new PIDController(0.0264, 0.132, 0.00348);// No overshoot
    rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
    leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);
    pid.enableContinuousInput(-180, 180);
    pid.setTolerance(0.5);

  }
  // Initial position of the robot is 100 inches away from the trap
  private static double yOffset= 100/12/3.28; // Converting 100 inches to meters

  public double getArmDistanceFromTheTarget(){
    double d;
    double x = m_drivetrainSubsystem.getPose().getX()*0.95;
    double y = m_drivetrainSubsystem.getPose().getY()*0.95 +yOffset;
    d= Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
    return d;
  }

  public double getTargetBotPosAngleRadians(){
    double x = m_drivetrainSubsystem.getPose().getX()*0.95;
    double y = m_drivetrainSubsystem.getPose().getY()*0.95 +yOffset;
    // Theta here is different than the theta in autonomous commands
    double theta = -Math.atan(x/y);
    return theta;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setFollowJoystick(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetRotation = Math.toDegrees(getTargetBotPosAngleRadians());
    SmartDashboard.putNumber("targetRotation", targetRotation);

    if (rightJoystick.getY()>0.05 || rightJoystick.getY()<-0.05) {
      yPowerCommanded = rightJoystick.getY() * -1;
    }

    if (rightJoystick.getX()>0.05 || rightJoystick.getX()<-0.05) {
      xPowerCommanded = rightJoystick.getX();
    }
    // yPowerCommanded=0;
    // xPowerCommanded=0;
    fbMotorPower = MathUtil.clamp(pid.calculate(m_drivetrainSubsystem.getPose().getRotation().getDegrees(), targetRotation),-1,1)*Math.PI;
    // if (pid.atSetpoint()) {
    //   fbMotorPower=0;
    // }
    SmartDashboard.putNumber("rotationSpeed", fbMotorPower);
    rotaionSpeed =fbMotorPower;
    m_drivetrainSubsystem.drive(
      -xPowerCommanded * m_drivetrainSubsystem.kMaxSpeed, 
      yPowerCommanded * m_drivetrainSubsystem.kMaxSpeed, 
      rotaionSpeed, 
      true);
      SmartDashboard.putNumber("Rotation error", MathUtil.clamp(targetRotation-m_drivetrainSubsystem.getPose().getRotation().getDegrees(),-25,25));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.setFollowJoystick(true);
    m_drivetrainSubsystem.setRotCommanded(0);
    // timer.stop();
    // timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (targetRotation+1>=m_drivetrainSubsystem.getPose().getRotation().getDegrees() && targetRotation-1<=m_drivetrainSubsystem.getPose().getRotation().getDegrees()) {
    //   timer.start();  
    // }
    // if (timer.get()>0.4) {
    //     return true;
    //   }

    // if (pid.atSetpoint()) {
    //   return true;
    // }
    return false;
  }
}
