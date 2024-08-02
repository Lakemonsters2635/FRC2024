// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public CANSparkMax intakeMotor;

  /* 
   * IntakeSubsystem is a constructor for IntakeSubsystem class.
   * All initialiation that applies to every single instance of this class goes here.
  */
  public IntakeSubsystem() {
    // Initialize the intake motor member variable as a CANSparkMax object with its motor id and the motor type. 

    // Set the current limit for this intake motor

    // Set whether or not the motor should be inverted

  }

  public void inIntake() {
    // Set the voltage for the intake in

  }

  public void outIntake() {
    // Set the voltage for the intake out

  }

  public void stopIntake() {
    // Set the voltage to stop the intake motor
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
