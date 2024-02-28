// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopeSubsystem extends SubsystemBase {
  public CANSparkMax telescopeMotor;
  public Encoder telescopeEncoder;
  private double currentEncoderCounts;
  private double ffMotorPower;
  private double theta;
  private Joystick lefJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);
  /** Creates a new TelescopeSubsystem. */
  public TelescopeSubsystem() {
    telescopeMotor = new CANSparkMax(Constants.TELESCOPE_MOTOR_ID, MotorType.kBrushless);
    telescopeEncoder = new Encoder(Constants.TELESCOPE_ENCODER_SOURCE_A, Constants.TELESCOPE_ENCODER_SOURCE_B);
  }

  public void extendTelescope() {
    telescopeMotor.setVoltage(lefJoystick.getThrottle() * 10);
  }

  public void retractTelescope() {
    telescopeMotor.set(Constants.TELESCOPE_RETRACT_SPEED);
  }

  public void stopTelescope() {

    telescopeMotor.set(Constants.TELESCOPE_STOP_SPEED);
  }

  public double getEncoderCounts(){
    return telescopeEncoder.get();
  }

  public void zeroEncoder (){
    telescopeEncoder.reset();
  }

  public double getTheta(){
    return telescopeEncoder.get()/23.5;
  }
  @Override
  public void periodic() {
    ffMotorPower = Constants.TELESCOPE_GAIN * Math.sin(Math.toRadians(getTheta()));
    // if(getEncoderCounts()>2100){
    // telescopeMotor.setVoltage(ffMotorPower);   
    // }
    telescopeMotor.setVoltage(ffMotorPower);
    //extendTelescope();
    SmartDashboard.putNumber("Telescope FF Power", ffMotorPower);
    SmartDashboard.putNumber("Telescope Scalded Units", getTheta());
    SmartDashboard.putNumber("Telescope Slider", lefJoystick.getThrottle());
    SmartDashboard.putNumber("Telescope Encoder Counts", telescopeEncoder.get());
    // This method will be called once per scheduler run
  }
}
