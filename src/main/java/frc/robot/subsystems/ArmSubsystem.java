// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
    public CANSparkMax m_armMotor1;
    public CANSparkMax m_armMotor2;
    public CANSparkMax m_armMotor3;
    public CANSparkMax m_armMotor4;

  public ArmSubsystem(){
    m_armMotor1 = new CANSparkMax(Constants.ARM_MOTOR1_ID, MotorType.kBrushless);
    m_armMotor2 = new CANSparkMax(Constants.ARM_MOTOR2_ID, MotorType.kBrushless);
    m_armMotor3 = new CANSparkMax(Constants.ARM_MOTOR3_ID, MotorType.kBrushless);
    m_armMotor4 = new CANSparkMax(Constants.ARM_MOTOR4_ID, MotorType.kBrushless);
  }

  public void ArmStart(){
    m_armMotor1.set(Constants.ARM_MOTOR_START_SPEED);
    m_armMotor2.set(Constants.ARM_MOTOR_START_SPEED);
    //m_armMotor3.set(Constants.ARM_MOTOR_Start_Speed);
    //m_armMotor4.set(Constants.ARM_MOTOR_Start_Speed);
  }

  public void ArmStop(){
  m_armMotor1.set(Constants.ARM_MOTOR_STOP_SPEED);
  m_armMotor2.set(Constants.ARM_MOTOR_STOP_SPEED);
  //m_armMotor3.set(Constants.ARM_MOTOR_Stop_Speed);
  //m_armMotor4.set(Constants.ARM_MOTOR_Stop_Speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
