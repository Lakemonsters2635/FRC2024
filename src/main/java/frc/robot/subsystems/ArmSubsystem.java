// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
    //Make member and instance variables Ex:motorPower, other motors, etc...
    private double ffMotorPower;
    private double gain;

  public ArmSubsystem(){
    //Initialize motors and put them on break mode
    
    //m_poseTarget=Constants.ARM_PICKUP_ANGLE;
  }

  

 

  public void setArmPower(double motorPower){
    //Give power to motors Hint: They go opposite directions

  }

  public double getTheta(){
    //Turn encoder counts into degrees

  }

  public boolean areWeThere(){
   //Figure out if the arm is close enough or is at the target position/angle or not
    
  }

  public void putToBoard(){
    //This method puts data to a program where you can see the numbers change live

    SmartDashboard.putNumber("Raw encoder value",m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Theta",theta);
   

  public void setPosTarget(double poseTarget){
    //set poseTarget in this method
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double upperLimit, lowerLimit;
    //m_poseTarget = m_poseTarget2;
    
      lowerLimit = Constants.ARM_LOWER_LIMIT;
      upperLimit = Constants.ARM_UPPER_LIMIT;
 
    
    //Periodic gets called every 20 miliseconds so putToBoard() is called to updata data
    putToBoard();
   

    gain = Constants.ARM_MOTOR_FF_GAIN;
    ffMotorPower = gain * Math.sin(Math.toRadians(theta));
    
    //Limits for feedback motor power so motors are not overworked
    double lowerLimitFB = -0.3; 
    double upperLimitFB = 0.3;

    //Set feedback motor power by using pid and clamps

    motorPower = ffMotorPower + fbMotorPower;
   
    //Set conditions so that if the arm is at some angles where it can rest it stops all motor power instead of wasting motor power
    
    //Clamps motorPower
    double clampVal = 3.0;

    //Using the clampVal variable clamp the motorPower
    motorPower = MathUtil.clamp();

    setArmPower(motorPower);
    SmartDashboard.putNumber("motorPower", motorPower);
    SmartDashboard.putNumber("m_poseTarget", m_poseTarget);

  }    
}