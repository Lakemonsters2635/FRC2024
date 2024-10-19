// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetRobotRot extends Command {
  /** Creates a new SetRobotRot. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  ObjectTrackerSubsystem m_objectTrackerSubsystem;
  PIDController pid;
  double rotationSpeed;
  double targetRotation;
  double yPowerCommanded;
  double xPowerCommanded;
  double targetRot;
  double deltaVisionAngle;
  Joystick rightJoystick;
  Joystick leftJoystick;
  private double fbMotorPower;
  public SetRobotRot(DrivetrainSubsystem drivetrainSubsystem, ObjectTrackerSubsystem objectTrackerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem =drivetrainSubsystem;
    // pid = new PIDController(0.09504, 0, 0.00528);// original PD
    // Higher kp makes it oscillate more as you drive because our robot naturally turns as it drives
    pid = new PIDController(0.05, 0, 0.005);// working PD
    // pid = new PIDController(0.079, 0.396, 0.00396);// Classic PID
    // pid = new PIDController(0.0264, 0.132, 0.00348);// No overshoot

    // TODO: remove this
    // this.targetRot = targetRot;

    m_objectTrackerSubsystem = objectTrackerSubsystem;
    // this.targetRotation = targetRot + m_drivetrainSubsystem.getPose().getRotation().getDegrees();

    // rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
    // leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);
    pid.enableContinuousInput(-180, 180);
    pid.setTolerance(0.5);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("INITIALIZE robot rotation !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    m_drivetrainSubsystem.setFollowJoystick(false);
    // TODO: this assumes we are using the closest april tag we need some mechanism to chose specific april tag
    deltaVisionAngle = Math.atan((double)m_objectTrackerSubsystem.visionX/m_objectTrackerSubsystem.visionZ);

    this.targetRotation = deltaVisionAngle + m_drivetrainSubsystem.getPose().getRotation().getRadians();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    deltaVisionAngle = Math.atan((double)m_objectTrackerSubsystem.visionX/m_objectTrackerSubsystem.visionZ);

    this.targetRotation = deltaVisionAngle + m_drivetrainSubsystem.getPose().getRotation().getRadians();

    // if (rightJoystick.getY()>0.05 || rightJoystick.getY()<-0.05) {
    //   yPowerCommanded = rightJoystick.getY() * -1;
    // }
    System.out.println("Set robot rotation !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    // if (rightJoystick.getX()>0.05 || rightJoystick.getX()<-0.05) {
    //   xPowerCommanded = rightJoystick.getX();
    // }
    // yPowerCommanded=0;
    // xPowerCommanded=0;
    fbMotorPower = MathUtil.clamp(pid.calculate(m_drivetrainSubsystem.getPose().getRotation().getRadians(), targetRotation)*180/Math.PI,-1.0,1.0);//*Math.PI;
    // if (pid.atSetpoint()) {
    //   fbMotorPower=0;
    // }

    SmartDashboard.putNumber("targetRotation", targetRotation);
    SmartDashboard.putNumber("rotationSpeed", fbMotorPower);
    SmartDashboard.putNumber("m_drivetrainSubsystem.getPose().getRotation().getRadiance()", m_drivetrainSubsystem.getPose().getRotation().getRadians());
    SmartDashboard.putNumber("pid.calculate(m_drivetrainSubsystem.getPose().getRotation().getRadians(), targetRotation)", pid.calculate(m_drivetrainSubsystem.getPose().getRotation().getRadians(), targetRotation));
    SmartDashboard.putNumber("deltaVisionAngle", deltaVisionAngle);

    rotationSpeed =fbMotorPower;
    // This is not exactly the angular velocity but the power inputed
    // m_drivetrainSubsystem.setRotCommanded(rotationSpeed);
    m_drivetrainSubsystem.drive(
      // -xPowerCommanded * m_drivetrainSubsystem.kMaxSpeed, 
      // yPowerCommanded * m_drivetrainSubsystem.kMaxSpeed, 
      0,
      0,
      rotationSpeed, 
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
