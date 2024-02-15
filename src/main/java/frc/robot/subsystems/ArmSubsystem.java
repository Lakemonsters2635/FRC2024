// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
    public CANSparkMax m_armMotor1;
    public CANSparkMax m_armMotor2;
    public static final AnalogInput m_encoder = new AnalogInput(Constants.ARM_ENCODER_ID); 
    private PIDController pid = new PIDController(0.0, 0.0, 0.0); 
    private double theta;
    private double m_poseTarget;
    private double fbMotorPower;
    private double ffMotorPower;

  public ArmSubsystem(){
    m_armMotor1 = new CANSparkMax(Constants.ARM_MOTOR1_ID, MotorType.kBrushless);
    m_armMotor2 = new CANSparkMax(Constants.ARM_MOTOR2_ID, MotorType.kBrushless);

    // m_armMotor2.setInverted(true);
    m_armMotor1.setInverted(true);
  }

  public void armStart(){
    m_armMotor1.set(RobotContainer.rightJoystick.getThrottle() * Constants.ARM_MOTOR_START_SPEED);
   m_armMotor2.set(RobotContainer.rightJoystick.getThrottle() * Constants.ARM_MOTOR_START_SPEED);

  }

  public void armStop(){
  m_armMotor1.set(Constants.ARM_MOTOR_STOP_SPEED);
  m_armMotor2.set(Constants.ARM_MOTOR_STOP_SPEED);
  }

  public void setArmPower(double motorPower){
    m_armMotor1.set(motorPower);
    m_armMotor2.set(-1*motorPower);
  }

  public double getArmDegrees(){
    return ((m_encoder.getValue()/4096.0))*(360);  //if get position gives encoder counts
    //((m_encoder.getPosition()%1)*1024)*(360) encoder counts if by rotations;
  }

  public void putToBoard(){
    SmartDashboard.putNumber("Calculated Degrees",getArmDegrees());
    SmartDashboard.putNumber("Raw encoder value",m_encoder.getValue());
    SmartDashboard.putNumber("Theta",theta);
    SmartDashboard.putNumber("Pose Target", m_poseTarget);
    SmartDashboard.putNumber("FB Power", fbMotorPower);
    SmartDashboard.putNumber("FF Power", ffMotorPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double upperLimit, lowerLimit;
    
      lowerLimit = Constants.ARM_LOWER_LIMIT;
      upperLimit = Constants.ARM_UPPER_LIMIT;
    // Arm pose trim override
    // Joystick hatJoystickTrimRotationArm = (Constants.HAT_JOYSTICK_TRIM_ROTATION_ARM == Constants.LEFT_JOYSTICK_CHANNEL)
    //   ? RobotContainer.leftJoystick
    //   : RobotContainer.rightJoystick;
    // if(hatJoystickTrimRotationArm.getPOV()==Constants.HAT_POV_ARM_UP){
    //   m_poseTarget += Constants.HAT_POSE_TARGET_PER_TIME_STEP;
    // }
    // if(hatJoystickTrimRotationArm.getPOV()==Constants.HAT_POV_ARM_DOWN){
    //   m_poseTarget += Constants.HAT_POSE_TARGET_PER_TIME_STEP*-1.0;
    // }
    m_poseTarget = MathUtil.clamp(m_poseTarget, lowerLimit, upperLimit);

    // manual control of the upper arm with z axis slider
    // double val = -RobotContainer.rightJoystick.getRawAxis(3);
    // double angle = (val+1.0)*180.0;
    // m_poseTarget = MathUtil.clamp(angle, lowerLimit, upperLimit);


    theta = 360.0 * (m_encoder.getValue() - Constants.ARM_ENCODER_OFFSET) / 4096.0;
    theta %= 360.0;
    if (theta < 0){
      theta += 360.0;
    }

    final double gain = Constants.ARM_MOTOR_FF_GAIN;
    ffMotorPower = gain * Math.sin(Math.toRadians(theta));

    double lowerLimitFB = -1; // TODO: fix these
    double upperLimitFB = 1;
    fbMotorPower = MathUtil.clamp(pid.calculate(theta, m_poseTarget), lowerLimitFB, upperLimitFB);

    double motorPower = fbMotorPower - ffMotorPower;
    // m_armMotor1.set(motorPower);
  }    
}
