// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopeSubsystem extends SubsystemBase {
  public CANSparkMax telescopeMotor;
  /** Creates a new TelescopeSubsystem. */
  public TelescopeSubsystem() {
    telescopeMotor = new CANSparkMax(Constants.TELESCOPE_MOTOR_ID, MotorType.kBrushless);
  }

  public void extendTelescope() {
    telescopeMotor.set(Constants.TELESCOPE_EXTEND_SPEED);
  }

  public void retractTelescope() {
    telescopeMotor.set(Constants.TELESCOPE_RETRACT_SPEED);
  }

  public void stopTelescope() {
    telescopeMotor.set(Constants.TELESCOPE_STOP_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
