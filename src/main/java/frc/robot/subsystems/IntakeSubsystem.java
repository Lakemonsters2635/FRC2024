// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public SparkMax intakeMotor;
  public SparkMaxConfig intakeMotorConfig;
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.idleMode(IdleMode.kBrake);
    intakeMotorConfig.smartCurrentLimit(20,1);
    intakeMotorConfig.inverted(true);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void inIntake() {
    intakeMotor.setVoltage(Constants.INTAKE_IN_SPEED * -10);
  }

  public void outIntake() {
    intakeMotor.setVoltage(Constants.INTAKE_OUT_SPEED * -10);
  }

  public void stopIntake() {
    intakeMotor.set(Constants.INTAKE_STOP_SPEED);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
