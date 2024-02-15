// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReleaseClimber extends SubsystemBase {
  /** Creates a new ReleaseClimber. */
  public static Servo climberReleaseServo1;
  public static Servo climberReleaseServo2;

  public ReleaseClimber() {
    climberReleaseServo1 = new Servo(Constants.SERVO_MOTOR_1);
    climberReleaseServo2 = new Servo(Constants.SERVO_MOTOR_2);
  }

  public void startReleasing(){
    climberReleaseServo1.set(Constants.FULL_EXTENDED_SERVO);
    climberReleaseServo2.set(Constants.FULL_EXTENDED_SERVO);
  }

  public void stopReleasing(){
    climberReleaseServo1.set(Constants.FULL_RETRACTED_SERVO);
    climberReleaseServo2.set(Constants.FULL_RETRACTED_SERVO);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
