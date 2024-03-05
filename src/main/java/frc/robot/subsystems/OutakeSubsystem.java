// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class OutakeSubsystem extends SubsystemBase {
  /** Creates a new OutakeSubsystem. */
  public CANSparkMax m_outakeMotor;

  public OutakeSubsystem() {
    m_outakeMotor = new CANSparkMax(Constants.OUTAKE_MOTOR_ID, MotorType.kBrushless);
  }

  
  public void setOutakePower(){
    m_outakeMotor.set(Constants.OUTTAKE_SPEED);
  }

  public void zeroOutakePower(){
    m_outakeMotor.set(Constants.OUTTAKE_STOP_SPEED);
  }

  public void manualOutake(){
    m_outakeMotor.set(0.6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
