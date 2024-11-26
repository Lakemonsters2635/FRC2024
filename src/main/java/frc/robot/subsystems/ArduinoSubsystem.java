// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArduinoSubsystem extends SubsystemBase {
  /** Creates a new ArduinoSubsystem. */
  DigitalInput leftRedIn = new DigitalInput(Constants.LEFT_RED_PIN);
  DigitalInput rightRedIn = new DigitalInput(Constants.RIGHT_RED_PIN);
  DigitalInput leftBlueIn = new DigitalInput(Constants.LEFT_BLUE_PIN);
  DigitalInput rightBlueIn = new DigitalInput(Constants.RIGHT_BLUE_PIN);

  public boolean leftRed = false;
  public boolean leftBlue = false;
  public boolean rightRed = false;
  public boolean rightBlue = false;
  DigitalOutput leftEnableOut = new DigitalOutput(Constants.LEFT_ENABLE_PIN);
  DigitalOutput rightEnableOut = new DigitalOutput(Constants.RIGHT_ENABLE_PIN);
  // sets the LED strip to be on by setting the Left and Right output PIN to be high
  public ArduinoSubsystem() {
      leftEnableOut.set(true);
      rightEnableOut.set(true);
  }
// This funciton toggels the value of leftEnableOut so that only one button is nessasary to turn the led strip of and off
  public void toggleLeftEnableOut(){
    if (leftEnableOut.get()){
      leftEnableOut.set(false);
    }
    else{
      leftEnableOut.set(true);
    }
  }
// The same for toggleRightEnableOut
  public void toggleRightEnableOut(){
      if (rightEnableOut.get()){
        rightEnableOut.set(false);
      }
      else{
        rightEnableOut.set(true);
      }
  }
  
  @Override
  public void periodic() {
    // output.set(false);
    // Timer.delay(0.5);
    // output.set(true);
    // Timer.delay(0.5);
    // //if (input.get()){
    //   System.out.println(input.get());
    //   if (input.get()){
    //     output.set(true);
    //   } else {
    //     output.set(false);
    //   }
    // This method will be called once per scheduler run
    // We set these variables equal to the input pin values for simplycity
    leftRed = leftRedIn.get();
    leftBlue = leftBlueIn.get();
    rightRed = rightRedIn.get();
    rightBlue = rightBlueIn.get();
    
    SmartDashboard.putBoolean("LR: ", leftRed);
    SmartDashboard.putBoolean("LB: ", leftBlue);
    SmartDashboard.putBoolean("RR: ", rightRed);
    SmartDashboard.putBoolean("RB: ", rightBlue);


    System.out.println("LR: " + leftRed);
    System.out.println("LB: " + leftBlue);
    System.out.println("RR: " + rightRed);
    System.out.println("RB: " + rightBlue);
  }
}
