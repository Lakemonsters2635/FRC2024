// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DrivetrainSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 3.63; // 3.63 meters per second
  public final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  private SwerveModuleState[] swerveModuleStates;

  public static Joystick rightJoystick = RobotContainer.rightJoystick;
  public static Joystick leftJoystick = RobotContainer.leftJoystick;

  public final double m_drivetrainWheelbaseWidth = 26.625 / Constants.INCHES_PER_METER;
  public final double m_drivetrainWheelbaseLength = 19.625 / Constants.INCHES_PER_METER;

  private Field2d field = new Field2d();

  // public final TelescopeExtendCommand m_telescopeExtendCommand = new TelescopeExtendCommand(null);

  // public final ScoreAmpCommand m_scoreAmpCommand = new ScoreAmpCommand(this, null, null);
  // public final ScoreShooterCommand m_scoreShooterCommand = new ScoreShooterCommand(this);
  // public final AprilTagChooser m_aprilTagChooser = new AprilTagChooser(m_scoreAmpCommand, m_scoreShooterCommand);
  // public final NoteTakerCommand m_noteTakerCommand = new NoteTakerCommand(this);
  // public final AutonomousCommand m_autonomousCommand = new AutonomousCommand(this, m_aprilTagChooser, m_noteTakerCommand);


    // public static boolean ignoreNote = true;

    // x is forward       robot is long in the x-direction, i.e. wheelbase length
    // y is to the left   robot is short in the y-direction, i.e. wheelbase width
    // robot front as currently labled on the motors (requires -x trajectory to go out into the +x field direction)
    public final Translation2d m_frontLeftLocation = 
            new Translation2d(m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);
    public final Translation2d m_frontRightLocation = 
            // TODO: Fix this... hack to swap FR BL
            // possibly we don't understand if z-axis is up or down?
            // new Translation2d(m_drivetrainWheelbaseLength/2, -m_drivetrainWheelbaseWidth/2);
            new Translation2d(-m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);
    public final Translation2d m_backLeftLocation = 
            // TODO: Fix this... hack to swap FR BL
            // possibly we don't understand if z-axis is up or down?
            // new Translation2d(-m_drivetrainWheelbaseLength/2, m_drivetrainWheelbaseWidth/2);
            new Translation2d(m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);
    public final Translation2d m_backRightLocation = 
            new Translation2d(-m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);
    public final Robot m_robot = new Robot();

    public final SwerveModule m_frontLeft = new SwerveModule(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER, 
                                                              Constants.FRONT_LEFT_ANGLE_OFFSET_COMPETITION,
                                                              1.0);
    public final SwerveModule m_frontRight = new SwerveModule(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER, 
                                                              Constants.FRONT_RIGHT_ANGLE_OFFSET_COMPETITION,
                                                              1.0);
    public final SwerveModule m_backLeft = new SwerveModule(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER, 
                                                              Constants.BACK_LEFT_ANGLE_OFFSET_COMPETITION,
                                                              1.0);
    public final SwerveModule m_backRight = new SwerveModule(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER, 
                                                              Constants.BACK_RIGHT_ANGLE_OFFSET_COMPETITION,
                                                              1.0);

    public final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
  
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation,
      m_frontRightLocation, 
      m_backLeftLocation, 
      m_backRightLocation);
  
    public final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
            });

  /** Creates a new DrivetrianSubsystem. */
  public DrivetrainSubsystem() {
    AutoBuilder.configureHolonomic(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              this::setDesiredStates, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
              new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                      new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                      4.5, // Max module speed, in m/s
                      0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                      new ReplanningConfig() // Default path replanning config. See the API for the options here
              ),
              () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
              },
              this // Reference to this subsystem to set requirements
    );
    
    PathPlannerLogging.setLogCurrentPoseCallback((poses) -> field.getObject("path").getPose());

    SmartDashboard.putData("field" ,field);
    
    getPose();
    zeroOdometry();
    resetAngle();
  }

  public Command createPathOnFlight(Pose2d targetPose, double endRot){
    // Create a list of bezier points from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            getPose(),
            targetPose
    );

    // Create the path using the bezier points created above
    PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(0.01, 0.01, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(endRot)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping =true;
    return AutoBuilder.followPath(path);
  }

  public Pose2d getTargetPosition(double rot){
    return new Pose2d(getPose().getX()+m_robot.x, getPose().getY()+m_robot.z, Rotation2d.fromDegrees(rot));
  }

  public double getTargetHeight(){
    //TODO: Figure out this method
    return m_robot.y-Constants.CAMERA_HEIGHT_FROM_GROUND;
  }

  public void resetAngle(){
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
  }

  private static double xPowerCommanded = 0;
  private static double yPowerCommanded = 0;
  private static double rotCommanded = 0;

  // three setters here and then call the setteres from the sd execute
  public static void setXPowerCommanded(double xPower) {
    xPowerCommanded = xPower;
  }

  public static void setYPowerCommanded(double yPower) {
    yPowerCommanded = yPower;
  }

  public static void setRotCommanded(double rot) {
    rotCommanded = rot;
  }

  @Override
  public void periodic() {
      //Hat Power Overides for Trimming Position and Rotation
      // System.out.println("getPose(): "+getPose());
      if(rightJoystick.getPOV()==Constants.HAT_POV_MOVE_FORWARD ){
        yPowerCommanded = Constants.HAT_POWER_MOVE;
      }
      else if(rightJoystick.getPOV()==Constants.HAT_POV_MOVE_BACK){
        yPowerCommanded = Constants.HAT_POWER_MOVE*-1.0;
      }
      else if(rightJoystick.getPOV()==Constants.HAT_POV_MOVE_RIGHT){
        xPowerCommanded = Constants.HAT_POWER_MOVE*1.0;
      }
      else if(rightJoystick.getPOV()==Constants.HAT_POV_MOVE_LEFT){
        xPowerCommanded = Constants.HAT_POWER_MOVE*-1.0;
      }

      if(leftJoystick.getPOV()==Constants.HAT_POV_ROTATE_RIGHT){
        rotCommanded = Constants.HAT_POWER_ROTATE*-1.0;
      }
      else if(leftJoystick.getPOV()==Constants.HAT_POV_ROTATE_LEFT){
        rotCommanded = Constants.HAT_POWER_ROTATE;
      }

      if (rightJoystick.getY()>0.05 || rightJoystick.getY()<-0.05) {
        yPowerCommanded = rightJoystick.getY() * -1;
      }

      if (rightJoystick.getX()>0.05 || rightJoystick.getX()<-0.05) {
        xPowerCommanded = rightJoystick.getX();
      }

      if (Math.pow(rightJoystick.getTwist(),3)>0.05 || Math.pow(rightJoystick.getTwist(),3)<-0.05) {
        rotCommanded = rightJoystick.getTwist() * -1;
      }
      
      this.drive(-xPowerCommanded * DrivetrainSubsystem.kMaxSpeed, 
                 yPowerCommanded * DrivetrainSubsystem.kMaxSpeed,
                 MathUtil.applyDeadband(-rotCommanded * this.kMaxAngularSpeed, 0.2), 
                 true);

      field.setRobotPose(getPose());
    
    double loggingState[] = {
        swerveModuleStates[0].speedMetersPerSecond,
        swerveModuleStates[0].angle.getDegrees(),
        swerveModuleStates[1].speedMetersPerSecond,
        swerveModuleStates[1].angle.getDegrees(),
        swerveModuleStates[2].speedMetersPerSecond,
        swerveModuleStates[2].angle.getDegrees(),
        swerveModuleStates[3].speedMetersPerSecond,
        swerveModuleStates[3].angle.getDegrees(),
      };

      SmartDashboard.putNumberArray("SwerveModuleStates",loggingState);


    
    updateOdometry();

    System.out.println("GetPose: " + getPose());

    putDTSToSmartDashboard();
    tuneAngleOffsetPutToDTS();
    // System.out.println("FL: " + m_frontLeft.printVoltage());
    // System.out.println("FR: " + m_frontRight.printVoltage());
  }

  public void recalibrateGyro() {
    System.out.println(m_gyro.getRotation2d());
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
    System.out.println(m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).   -1.0 ... +1.0
   * @param ySpeed Speed of the robot in the y direction (sideways).  -1.0 ... +1.0
   * @param rot Angular rate of the robot.                            -1.0 ... +1.0
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d().unaryMinus())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  /** Get pose from odometry field **/
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return m_kinematics; 
  }

  /** zeroes drivetrain odometry **/
  public void zeroOdometry() {
    resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/DriveSubsystem.java
   * 
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

  /** Sets the swerve ModuleStates.
   * @param cs The desired SwerveModule states as a ChassisSpeeds object
   */
  private void setDesiredStates(ChassisSpeeds cs) {
    SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(cs);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, 4);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  } 

  /** Sets the swerve ModuleStates. Accept a center of rotation for when you DON'T want to rotate
   * around the center of the robot
   * @param cs The desired SwerveModule states as a ChassisSpeeds object
   * @param centerOfRotation Center of rotation. Ex. location of camera
   */
  private void setDesiredStates(ChassisSpeeds cs, Translation2d centerOfRotation) {
    // System.out.println("vX: " + Math.round(cs.vxMetersPerSecond*100.0)/100.0 + "  vY: " + Math.round(cs.vyMetersPerSecond));
    SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(cs, centerOfRotation);

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 4);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  } 

public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState());
    return chassisSpeeds;
  }

  public AHRS getGyroscope() {
    return m_gyro; 
  }

  /**Sets the swerve ModuleStates.
   * @param desiredStates The desired SwerveModule states. Array of `SwerveModuleState[]`
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DrivetrainSubsystem.kMaxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }
  
  /** Displays all 4 module positions + robot pose (forward/back) in SmartDashboard. 
   * </p> For debugging
   */
  public void putDTSToSmartDashboard() {}

  /**
   * Procedure for tuning:
   * </p>   1. Put tuneAngleOffsetPutToDTS() in periodic(). 
   * </p>   2. Read the angles when the wheels are lined up. 
   * </p>   3. Add/subtract degrees from the angle offsets in Constants until they all read 0/pi/2pi when perfectly lined up
   */
  public void tuneAngleOffsetPutToDTS() {
    // TUNE ANGLE OFFSETS
    
    // SmartDashboard.putNumber("FL encoder pos", Math.toDegrees(m_frontLeft.getTurningEncoderRadians()));
    // SmartDashboard.putNumber("FR encoder pos", Math.toDegrees(m_frontRight.getTurningEncoderRadians()));
    // SmartDashboard.putNumber("BL encoder pos", Math.toDegrees(m_backLeft.getTurningEncoderRadians()));
    // SmartDashboard.putNumber("BR encoder pos", Math.toDegrees(m_backRight.getTurningEncoderRadians())); 

    // SmartDashboard.putNumber("FL SMS Speed", swerveModuleStates[0].speedMetersPerSecond);
    // SmartDashboard.putNumber("FL SMS Angle", swerveModuleStates[0].angle.getDegrees());


    // SmartDashboard.putNumber("FR SMS Speed", swerveModuleStates[1].speedMetersPerSecond);
    // SmartDashboard.putNumber("FR SMS Angle", swerveModuleStates[1].angle.getDegrees());

    // SmartDashboard.putNumber("BL SMS Speed", swerveModuleStates[2].speedMetersPerSecond);
    // SmartDashboard.putNumber("BL SMS Angle", swerveModuleStates[2].angle.getDegrees());

    // SmartDashboard.putNumber("BR SMS Speed", swerveModuleStates[3].speedMetersPerSecond);
    // SmartDashboard.putNumber("BR SMS Angle", swerveModuleStates[3].angle.getDegrees());
    
    // SmartDashboard.putNumber("Gyro Rotation 2d",m_gyro.getRotation2d().getDegrees());
    // SmartDashboard.putNumber("Gyro Speed X",m_gyro.getVelocityX());
    // SmartDashboard.putNumber("Gyro Speed Y",m_gyro.getVelocityY());

    SmartDashboard.putString("GetPose:", "("+Double.toString(getPose().getX())+" , "+Double.toString(getPose().getY())+" , "+Double.toString(getPose().getRotation().getDegrees())+")");


  }
}