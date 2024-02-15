// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
    public CANSparkMax m_armMotor1;
    public CANSparkMax m_armMotor2;

  public ArmSubsystem(){
    m_armMotor1 = new CANSparkMax(Constants.ARM_MOTOR1_ID, MotorType.kBrushless);
    m_armMotor2 = new CANSparkMax(Constants.ARM_MOTOR2_ID, MotorType.kBrushless);

    m_armMotor2.setInverted(true);
    // m_armMotor1.setInverted(true);
  }

  public void armStart(){
    m_armMotor1.set(RobotContainer.rightJoystick.getThrottle() * Constants.ARM_MOTOR_START_SPEED);
    m_armMotor2.set(RobotContainer.rightJoystick.getThrottle() * Constants.ARM_MOTOR_START_SPEED);
  }

  public void armStop(){
  m_armMotor1.set(Constants.ARM_MOTOR_STOP_SPEED);
  m_armMotor2.set(Constants.ARM_MOTOR_STOP_SPEED);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
