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

    // FRONT LEFT
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 8; //1
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 1; //3
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 7; //2
    public static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(49.73+3-1.1+3-6.2-0.5+180); //3.0346

    //    public static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(265.96+5.1-13+3+3.8+5.0); //3.01

    // FRONT RIGHT
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 9; //3
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 0; //2
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 10; //4
    public static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(151.84+0.6-3.5+3+0.6+1.5+180); //2.9835

    //    public static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(17.07+5+50-90-19.3+3+4.8); // 3.0775

    // BACK LEFT
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 3; //9
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 2; //0
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 4; //10
    public static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(17.07+5+50-90-19.3+3+4.8+180); // 3.0775

    //    public static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(151.84+0.6-3.5+3+0.6+1.5); //2.9835

    // BACK RIGHT
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 1; //8
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 3;//1
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 2; //7
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

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

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

    // CLIMBER CONSTANTS
    public static final int CLIMBER_MOTOR_1 = 6;
    public static final int CLIMBER_MOTOR_2 = 5;

    public static final int CLIMBER_ENCODER_1 = 6;
    public static final int CLIMBER_ENCODER_2 = 5;

    public static final double CLIMBER_1_START_SPEED = 0.4;
    public static final double CLIMBER_2_START_SPEED = 0.4;

    public static final int CLIMBER_STOP_SPEED = 0;

    // AUTO CONSTANTS
    public static final double DISTANCE_BETWEEN_NOTES = Units.inchesToMeters(54); // 57
    public static final double DISTANCE_TO_NOTE = Units.inchesToMeters(35+6)+0.1; //45

    public static final double RIGHT_ENDING_POSE = 45+10;
    public static final double LEFT_ENDING_POSE = -53-8;

    // CLIMBER RELEASE/SERVO CONSTANTS // TODO: Delete these
    public static final int SERVO_MOTOR_1 = 0;
    public static final int SERVO_MOTOR_2 = 1;

    public static final double FULL_EXTENDED_SERVO = 1.0;
    public static final double FULL_RETRACTED_SERVO = 0.0;

    //ARM CONSTANT: 15 is right, 21 is left
    public static final int ARM_MOTOR1_ID = 21; // note to self: next time say left and right
    public static final int ARM_MOTOR2_ID = 15;
    public static final int ARM_ENCODER_ID = 0;

    public static final double ARM_MOTOR_START_SPEED = 0.3;
    public static final int ARM_MOTOR_STOP_SPEED = 0;

    public static final double ARM_LOWER_LIMIT = -30; //TODO: figure out these
    public static final double ARM_UPPER_LIMIT = 93.3;

    //    public static final double ARM_MOTOR_FF_GAIN = -1.4; // 12V
    //    public static final double ARM_MOTOR_FF_GAIN = -1.16; // 11V // For Neo 500
    public static final double ARM_MOTOR_FF_GAIN = -0.59; // 11V // For Krakens

    public static final double ARM_ENCODER_OFFSET = 0.406; 
    public static final int ARM_PICKUP_ANGLE = 94; // 104
    public static final int ARM_AMP_ANGLE = -18; // -19
    public static final int ARM_SHOOTER_ANGLE = 52; // 54
    public static final int ARM_SHOOTER_ANGLE_MID_AUTO = 34; // 34
    public static final int ARM_SHOOTER_ANGLE_SIDE_AUTO = 33; // 34
    public static final int ARM_TRAP_SHOOT_ANGLE = 71; // 72


    // INTAKE CONSTANTS
    public static final int INTAKE_MOTOR_ID = 13;

    public static final int INTAKE_STOP_SPEED = 0;
    public static final double INTAKE_IN_SPEED = -0.9;
    public static final double INTAKE_OUT_SPEED = 0.2;


    // OUTTAKE CONSTANTS
    public static final int OUTAKE_MOTOR_ID = 14;

    public static final double OUTTAKE_STOP_SPEED =0;
    public static final double OUTTAKE_SPEED = -0.95;
    public static final double AMP_OUTTAKE_SPEED = -0.4;
    public static final double OUTTAKE_TRAP_SPEED = -0.15;
    public static final double OUTTAKE_IN_SPEED = 0.35;

    // TELESCOPE CONSTANTS
    public static final int TELESCOPE_MOTOR_ID = 11;

    public static final double TELESCOPE_EXTEND_SPEED = -0.4;
    public static final double TELESCOPE_RETRACT_SPEED = 0.4;
    public static final int TELESCOPE_STOP_SPEED = 0;

    public static final int TELESCOPE_ENCODER_ID1 = 0;
    public static final int TELESCOPE_ENCODER_ID2 = 1;

    // AUTOMOVESWERVE CONSTANTS
    public static final double CHANGE_IN_X_PER_SECOND= 0.714;
    public static final double CHANGE_IN_Y_PER_SECOND= 0.717;

    // VISION CONSTANTS
    public static final double VISION_NOTE_CAM_TILT = Units.degreesToRadians(54);
    public static final double VISION_APRIL_TAG_PRO_TILT = Units.degreesToRadians(0);

    public static final int TARGET_TRIGGER_DISTANCE_APRIL_TAG = 0;

    //BUTTON BINDINGS

    // right buttons
    public static final int INTAKE_BUTTON = 1;
    public static final int GROUND_PICKUP_BUTTON = 3;
    public static final int AMP_POSE_BUTTON = 4;

    public static final int INTAKE_OUT_BUTTON = 5;   
    public static final int SPEAKER_BUTTON = 6;
    public static final int SWERVE_RESET_BUTTON = 7;
    public static final int OUTTAKE_IN_BUTTON = 8;
    public static final int TRAP_SHOOT_BUTTON = 12;
    // public static final int SPEAKER_POSE_BUTTON = 6;
    public static final int BALANCE_BUTTON = 10;

    // left buttons
    public static final int OUTTAKE_BUTTON = 1;
    public static final int FAR_SHOOTER_BUTTON = 4;
    //public static final int TELESCOPE_RETRACT_BUTTON = 4;
    //public static final int TELESCOPE_EXTEND_BUTTON = 6;
    public static final int CLIMBER_UP_BUTTON = 7;
    public static final int CLIMBER_DOWN_BUTTON = 8;
    public static final int CLIMBER1_UP_BUTTON = 9;
    public static final int CLIMBER2_UP_BUTTON = 10;
    public static final int CLIMBER1_DOWN_BUTTON = 11;
    public static final int CLIMBER2_DOWN_BUTTON = 12;


    //public static final int ARM_START_BUTTON = 5;
}
