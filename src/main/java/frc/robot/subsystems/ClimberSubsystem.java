// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public CANSparkMax m_climberMotor1;
  public CANSparkMax m_climberMotor2;

  public ClimberSubsystem(){
    m_climberMotor1 = new CANSparkMax(Constants.CLIMBER_MOTOR_1, MotorType.kBrushless);
    m_climberMotor2 = new CANSparkMax(Constants.CLIMBER_MOTOR_2, MotorType.kBrushless);
    
    m_climberMotor1.setIdleMode(IdleMode.kBrake);
    m_climberMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void climberUp(){
    // m_climberMotor1.set(Constants.CLIMBER_1_START_SPEED);
    // m_climberMotor2.set(Constants.CLIMBER_2_START_SPEED);
    climberUp1();
    climberUp2();
  }
  
  public void climberDown(){
    // m_climberMotor1.set(Constants.CLIMBER_1_START_SPEED * -1);
    // m_climberMotor2.set(Constants.CLIMBER_2_START_SPEED * -1);
    climberDown1();
    climberDown2();
  }

  public void stopClimber(){
    m_climberMotor1.set(Constants.CLIMBER_STOP_SPEED);
    m_climberMotor2.set(Constants.CLIMBER_STOP_SPEED);  
  }

  public void climberUp1(){
    m_climberMotor1.set(Constants.CLIMBER_1_START_SPEED);
  }
  
  public void climberUp2(){
    m_climberMotor2.set(Constants.CLIMBER_2_START_SPEED);
  }

  public void climberDown1(){
    m_climberMotor1.set(Constants.CLIMBER_1_START_SPEED * -1);
  }
  
  public void climberDown2(){
    m_climberMotor2.set(Constants.CLIMBER_2_START_SPEED * -1);
  }

  public void stopClimber1(){
    m_climberMotor1.set(Constants.CLIMBER_STOP_SPEED);
  }

  public void stopClimber2(){
    m_climberMotor2.set(Constants.CLIMBER_STOP_SPEED);  
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
