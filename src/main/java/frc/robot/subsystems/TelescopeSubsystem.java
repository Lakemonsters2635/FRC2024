// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopeSubsystem extends SubsystemBase {
  public CANSparkMax telescopeMotor;
  public Encoder telescopeEncoder;
  /** Creates a new TelescopeSubsystem. */
  public TelescopeSubsystem() {
    telescopeMotor = new CANSparkMax(Constants.TELESCOPE_MOTOR_ID, MotorType.kBrushless);
    telescopeEncoder = new Encoder(Constants.TELESCOPE_ENCODER_ID1, Constants.TELESCOPE_ENCODER_ID2);
    telescopeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void extendTelescope() {
    telescopeMotor.set(Constants.TELESCOPE_EXTEND_SPEED);
  }

  public double getEncoderCounts(){
    return telescopeEncoder.get();
  }

  public void resetEncoder(){
    telescopeEncoder.reset();
  }

  public void retractTelescope() {
    telescopeMotor.set(Constants.TELESCOPE_RETRACT_SPEED);
  }

  public void stopTelescope() {
    telescopeMotor.set(Constants.TELESCOPE_STOP_SPEED);
  }

  @Override
  public void periodic() {
    if(getEncoderCounts()>1920){
      stopTelescope();
    }
    
    // This method will be called once per scheduler run
  }
}
