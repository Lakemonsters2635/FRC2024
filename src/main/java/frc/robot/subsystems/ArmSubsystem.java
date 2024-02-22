// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
    public TalonFX m_armMotor1;
    public TalonFX m_armMotor2;
    public static final AnalogInput m_encoder = new AnalogInput(Constants.ARM_ENCODER_ID); 
    // private PIDController pid = new PIDController(0.108, 0.0, 0.024); 
    private PIDController pid = new PIDController(0.009, 0.0, 0.002); 
    private double theta;
    public double m_poseTarget;
    private double fbMotorPower;
    private double ffMotorPower;
    private double motorPower;
    //public double m_poseTarget2=80;
    Joystick rightJoystick = new Joystick(0);
    double gain;

  public ArmSubsystem(){
    m_armMotor1 = new TalonFX(Constants.ARM_MOTOR1_ID);
    m_armMotor2 = new TalonFX(Constants.ARM_MOTOR2_ID);

    m_armMotor1.setNeutralMode(NeutralModeValue.Brake);
    m_armMotor2.setNeutralMode(NeutralModeValue.Brake);

    m_poseTarget = Constants.ARM_AMP_ANGLE;
  }

  public void controlArmThrottle(){
    m_poseTarget = MathUtil.clamp(RobotContainer.rightJoystick.getThrottle()*180, Constants.ARM_LOWER_LIMIT, Constants.ARM_UPPER_LIMIT);
  }

  public void armStop(){
    m_armMotor1.set(Constants.ARM_MOTOR_STOP_SPEED);
    m_armMotor2.set(Constants.ARM_MOTOR_STOP_SPEED);
  }

  public void setArmPower(double motorPower){
    m_armMotor1.setVoltage(motorPower*-1);
    m_armMotor2.setVoltage(1*motorPower);
  }

  public double getArmDegrees(){
    return ((m_encoder.getValue()/4096.0))*(360);
  }

  public double getTheta(){
    theta = 360.0 * (m_encoder.getValue() - Constants.ARM_ENCODER_OFFSET) / 4096.0;
    theta %= 360.0;
    if (theta < 0){
      theta += 360.0;
    }
    if (theta > 180){
      theta -= 360;
    }
    return theta;
  }

  public boolean areWeThere(){
    double currentTheta = getTheta();
    if (currentTheta > (m_poseTarget - 4) || currentTheta > (m_poseTarget + 4)){
      return true;
    } else{
      return false;
    }
  }

  public void putToBoard(){
    SmartDashboard.putNumber("Calculated Degrees",getArmDegrees());
    SmartDashboard.putNumber("Raw encoder value",m_encoder.getValue());
    SmartDashboard.putNumber("Theta",theta);
    SmartDashboard.putNumber("Pose Target", m_poseTarget);
    SmartDashboard.putNumber("FB Power", fbMotorPower);
    SmartDashboard.putNumber("FF Power", ffMotorPower);
    SmartDashboard.putNumber("Motor Power", motorPower);
    SmartDashboard.putNumber("Input", RobotContainer.rightJoystick.getThrottle());
    SmartDashboard.putNumber("Gain", gain);
    SmartDashboard.putNumber("Slider Angle", MathUtil.clamp(RobotContainer.rightJoystick.getThrottle()*180, Constants.ARM_LOWER_LIMIT, Constants.ARM_UPPER_LIMIT));
  }

  public void setPosTarget(double poseTarget){
    SmartDashboard.putNumber("poseTarget parameter", poseTarget);
    m_poseTarget = poseTarget;
    System.out.println("MOVE ARM POS SEt");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double upperLimit, lowerLimit;
    //m_poseTarget = m_poseTarget2;
    
      lowerLimit = Constants.ARM_LOWER_LIMIT;
      upperLimit = Constants.ARM_UPPER_LIMIT;
    // Arm pose trim override
    Joystick hatJoystickTrimRotationArm = RobotContainer.leftJoystick;
    if(hatJoystickTrimRotationArm.getPOV()==Constants.HAT_POV_ARM_UP){
      m_poseTarget += Constants.HAT_POSE_TARGET_PER_TIME_STEP;
    }
    if(hatJoystickTrimRotationArm.getPOV()==Constants.HAT_POV_ARM_DOWN){
      m_poseTarget += Constants.HAT_POSE_TARGET_PER_TIME_STEP*-1.0;
    }
    m_poseTarget = MathUtil.clamp(m_poseTarget, lowerLimit, upperLimit);

    // manual control of the upper arm with z axis slider
    // double val = -RobotContainer.rightJoystick.getRawAxis(3);
    // double angle = (val + 1.0) * 180.0;
    // m_poseTarget = MathUtil.clamp(angle, lowerLimit, upperLimit);

    
    getTheta();
    putToBoard();
    // controlArmThrottle();

    gain = Constants.ARM_MOTOR_FF_GAIN;
    //gain =rightJoystick.getThrottle()*0.3;
    ffMotorPower = gain * Math.sin(Math.toRadians(theta));

    double lowerLimitFB = -0.3; // TODO: fix these
    double upperLimitFB = 0.3;

    // TODO: ensure both PID and FF are in voltage mode.
    fbMotorPower = MathUtil.clamp(pid.calculate(theta, m_poseTarget), lowerLimitFB, upperLimitFB) * 10;

    motorPower = ffMotorPower + fbMotorPower;
    if(m_poseTarget>102 && theta > 100){
      motorPower = 0;
    }
    setArmPower(motorPower);

  }    
}
