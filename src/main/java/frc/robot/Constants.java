// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // joystick channels
    public static final int RIGHT_JOYSTICK_CHANNEL = 1;
    public static final int LEFT_JOYSTICK_CHANNEL = 0;

    public static final double INCHES_PER_METER = 39.37;

    // If you want to change the definition of what is front left etc. 

    // For 180 switch the constants of encoder offsets, motor and encoder ids for front left and back right, and front right and back left
    // Add 180 to all the encoder offsets

    // For rotating front of the robot clockwise 90 degrees for motor and encoder constants and encoder offsets
    // make the front left to front right, front right to back right, back right to back left, back left to front left
    // Subtract 90 to all encoder offsets
    // Switch robot's lenght to width

    // For rotating front of the robot counter-clockwise 90 degrees for motor and encoder constants and encoder offsets
    // make the front left to back left, back left to back right, back right to front right, front right to front left
    // Add 90 to all encoder offsets
    // Switch robot's lenght to width

    // ROBOT WIDTH AND LENGHT
    public static final double DRIVETRAIN_WHEELBASE_WIDTH =  21.5/ Constants.INCHES_PER_METER; //21.5 Calibrated for 2024 BunnyBots
    public static final double DRIVETRAIN_WHEELBASE_LENGTH = 25.5 / Constants.INCHES_PER_METER; //25.5 Calibrated for 2024 BunnyBots


    // FRONT LEFT
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1; //1
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; //3
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 2; //2
    public static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(49.73+3-1.1+3-6.2-0.5+180); //3.0346

    //    public static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(265.96+5.1-13+3+3.8+5.0); //3.01

    // FRONT RIGHT
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 7; //7
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 0; //2
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 8; //8
    public static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(151.84+0.6-3.5+3+0.6+1.5+180); //2.9835

    //    public static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(17.07+5+50-90-19.3+3+4.8); // 3.0775

    // BACK LEFT
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 3; //3
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 0; //0
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 4; //4
    public static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(17.07+5+50-90-19.3+3+4.8+180); // 3.0775

    //    public static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(151.84+0.6-3.5+3+0.6+1.5); //2.9835

    // BACK RIGHT
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 5; //5
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 0;//1
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 6; //6
    public static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(265.96+5.1-13+3+3.8+5.0+180); //3.01

    //    public static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(49.73+3-1.1+3-6.2-0.5); //3.0346

    // hat constants 
    public static final int HAT_JOYSTICK_TRIM_POSITION = RIGHT_JOYSTICK_CHANNEL;
    public static final int HAT_JOYSTICK_TRIM_ROTATION_ARM = LEFT_JOYSTICK_CHANNEL;
    public static final double HAT_POWER_MOVE = 0.1;
    public static final double HAT_POWER_ROTATE = 0.3;
    // Hat trim target speed is 15 degrees per second
    // One time step is 0.02 seconds
    // 0.3 degrees per time step is our target change when the hat is active
    public static final double HAT_POSE_TARGET_PER_TIME_STEP = -0.3; // negative is raising the arm
    public static final int HAT_POV_MOVE_LEFT = 270;
    public static final int HAT_POV_MOVE_RIGHT = 90;
    public static final int HAT_POV_MOVE_FORWARD = 0;
    public static final int HAT_POV_MOVE_BACK = 180;
    public static final int HAT_POV_ARM_UP = 0;
    public static final int HAT_POV_ARM_DOWN = 180;
    public static final int HAT_POV_ROTATE_LEFT = 270;
    public static final int HAT_POV_ROTATE_RIGHT = 90;
    /*Changed max speed from 2pi rad/s since max no load speed is 15 ft/s and with our robot
    the radius from the center of our robot to the center of the module is 16.5 inches therefore
    with a max commanded angular speed of 2pi radians our max commanded speed is 8.6 ft/s which
    is about half the theoretical maximum. We don't want to go to the absolute maximum which would
    approximately be 4pi rad/s so we choose 3pi rad/s as the maximum.
    Note: if we run into problems during autos with it lagging, we may want to revert it back to 2pi rad/s. */
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI; 
    //We set it to 9pi rad/s^2 so that we get to the maximum speed within 1/3 of a second
    //We may want to back off on this if autos start lagging
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 12 * Math.PI;

    public static final double maxModuleLinearSpeed = 1.75;
    public static final double maxModuleLinearAccelaration = 8;
    // public static final double maxModuleLinearSpeed = 3.5;
    // public static final double maxModuleLinearAccelaration = 16;

    public static final int kEncoderCPR = 42; // neo encoder ticks per revolution
    public static final double kWheelDiameterMeters = 4.0 / 39.37;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) * (1.0 / (45.0 / 15.0) / (17.0 / 27.0) / (50.0 / 14.0));  //Mark 4i L2 Gear Ratio 
    // put into manual mode, manually read position and rotate wheel

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 0.5;

    public static final double kPModuleDriveController = 0;

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // VISION CONSTANTS
    public static double OBJECT_DETECTION_LATENCY = 0.217; // seconds
    public static final double VISION_NOTE_CAM_TILT = Units.degreesToRadians(0);
    public static final double VISION_APRIL_TAG_PRO_TILT = Units.degreesToRadians(0);

    public static final int TARGET_TRIGGER_DISTANCE_APRIL_TAG = 0;

    // AUTO CONSTANTS
    
    // AUTOMOVESWERVE CONSTANTS
    public static final double CHANGE_IN_X_PER_SECOND= 0.714;
    public static final double CHANGE_IN_Y_PER_SECOND= 0.717;

    //BUTTON BINDINGS

    // right buttons
    public static final int SWERVE_RESET_BUTTON = 7;
    public static final int BALANCE_BUTTON = 10;

    // left buttons
    
}
