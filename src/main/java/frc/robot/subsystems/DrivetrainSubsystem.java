// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
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

    // x is forward       robot is long in the x-direction, i.e. wheelbase length
    // y is to the left   robot is short in the y-direction, i.e. wheelbase width
    // robot front as currently labled on the motors (requires -x trajectory to go out into the +x field direction)
    public final Translation2d m_frontLeftLocation = 
            new Translation2d(m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);
    public final Translation2d m_frontRightLocation = 
            new Translation2d(-m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);
    public final Translation2d m_backLeftLocation = 
            new Translation2d(m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);
    public final Translation2d m_backRightLocation = 
            new Translation2d(-m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);

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
    
    public boolean followJoystics = true;
  
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
    // TODO: Delete this if don't needed
    AutoBuilder.configureHolonomic(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              this::setDesiredStates, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
              new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                      new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                      4.5, // Max module speed, in m/s
                      0.42, // Drive base radius in meters. Distance from robot center to furthest module.
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

  /**
   * Go to targetPose using pathplanner
   * @param targetPose
   * @return command to make robot go to targetPose
   */
  public Command goToTargetPos(Pose2d targetPose){
    // System.out.println("Target pos: "+"x:"+targetPose.getX()+" y:"+targetPose.getY()+" degrees:"+targetPose.getRotation().getDegrees());

    // Since we are using a holonomic drivetrain, the rotation component of this pose
    // represents the goal holonomic rotation

    // Create the constraints to use while pathfinding
    // PathConstraints constraints = new PathConstraints(
    //         0.1, 0.1,
    //         Units.degreesToRadians(10), Units.degreesToRadians(10));

    // // See the "Follow a single path" example for more info on what gets passed here
    // Command pathfindingCommand = new PathfindHolonomic(
    //         targetPose,
    //         constraints,
    //         0.0, // Goal end velocity in m/s. Optional
    //         this::getPose,
    //         this::getChassisSpeeds,
    //         this::setDesiredStates,
    //         new HolonomicPathFollowerConfig(4.5,0.42,new ReplanningConfig()), // TODO: Figure out these numbers
    //         0.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
    //         this // Reference to drive subsystem to set requirements
    // );

    // return pathfindingCommand;
    
    // Create a list of bezier points from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            getPose(),
            targetPose
    );


    // Create the path using the bezier points created above
    PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    PathPlannerTrajectory traj = new PathPlannerTrajectory(
            path,
            getChassisSpeeds(),
            getPose().getRotation()
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping =true;

    Command followPathcCommand = new FollowPathHolonomic(
            path, 
            this::getPose, 
            this::getChassisSpeeds, 
            this::setDesiredStates, 
            new HolonomicPathFollowerConfig(4.5,0.42,new ReplanningConfig()),
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
            this
    );
    
    return followPathcCommand;
  }

  public void stopMotors(){
    m_backLeft.stop();
    m_frontLeft.stop();
    m_backRight.stop();
    m_frontRight.stop();
  }

  public Command pathChooser(String autoName){

    PathPlannerAuto c = new PathPlannerAuto(autoName);

    // Command followPathcCommand = new FollowPathHolonomic(
    //         path, 
    //         this::getPose, 
    //         this::getChassisSpeeds, 
    //         this::setDesiredStates, 
    //         new HolonomicPathFollowerConfig(4.5,0.42,new ReplanningConfig()),
    //         () -> {
    //               // Boolean supplier that controls when the path will be mirrored for the red alliance
    //               // This will flip the path being followed to the red side of the field.
    //               // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //               var alliance = DriverStation.getAlliance();
    //               if (alliance.isPresent()) {
    //                 return alliance.get() == DriverStation.Alliance.Red;
    //               }
    //               return false;
    //         },
    //         this
    // );

    return c;
  }

  public Command createPath(){
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      0.5, 
      0.5)//TODO
      .setKinematics(m_kinematics);

    edu.wpi.first.math.trajectory.Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)),
      List.of(
        new Translation2d(0,0.5)
        // new Translation2d(-0.5,0.5)
      ),
      new Pose2d(0,1,new Rotation2d(0)),
      trajectoryConfig
      );

      TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Constants.kMaxModuleAngularSpeedRadiansPerSecond, Constants.kMaxModuleAngularAccelerationRadiansPerSecondSquared);

      PIDController xController = new PIDController(0, 0, 0);
      PIDController yController = new PIDController(0, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0, kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        this::getPose,
        m_kinematics,
        xController,
        yController,
        thetaController,
        this::setModuleStates,
        this
      );

      return swerveControllerCommand;
  }

  public void resetAngle(){
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
  }

  private static double xPowerCommanded = 0;
  private static double yPowerCommanded = 0;
  private static double rotCommanded = 0;

  // three setters here and then call the setteres from the sd execute
  public void setXPowerCommanded(double xPower) {
    xPowerCommanded = xPower;
  }

  public void setYPowerCommanded(double yPower) {
    yPowerCommanded = yPower;
  }

  public void setRotCommanded(double rot) {
    rotCommanded = rot;
  }

  @Override
  public void periodic() {
      //Hat Power Overides for Trimming Position and Rotation
      // System.out.println("Current pos: "+"x:"+getPose().getX()+" y:"+getPose().getY()+" degrees:"+getPose().getRotation().getDegrees());
      if (followJoystics) {
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
        
      }
      else{
      }

      
      this.drive(-xPowerCommanded * DrivetrainSubsystem.kMaxSpeed, 
                yPowerCommanded * DrivetrainSubsystem.kMaxSpeed,
                MathUtil.applyDeadband(-rotCommanded * this.kMaxAngularSpeed, 0.2), 
                true);
     
      SmartDashboard.putNumber("rotCommanded", rotCommanded);

      double loggingState[] = {     //Array for predicted values
        swerveModuleStates[3].angle.getDegrees(),
        swerveModuleStates[3].speedMetersPerSecond,
        swerveModuleStates[1].angle.getDegrees(),
        swerveModuleStates[1].speedMetersPerSecond,
        swerveModuleStates[2].angle.getDegrees(),
        swerveModuleStates[2].speedMetersPerSecond,
        swerveModuleStates[0].angle.getDegrees(),
        swerveModuleStates[0].speedMetersPerSecond,
      };

      double actualLoggingState[] = {
        m_frontLeft.getTurningEncoderRadians() * 180 / Math.PI,
        m_frontLeft.getVelocity(),
        m_frontRight.getTurningEncoderRadians() * 180 / Math.PI,
        m_frontRight.getVelocity(),
        m_backLeft.getTurningEncoderRadians() * 180 / Math.PI,
        m_backLeft.getVelocity(),
        m_backRight.getTurningEncoderRadians() * 180 / Math.PI,
        m_backRight.getVelocity(),
      };

      SmartDashboard.putNumberArray("SwerveModuleStates",loggingState);
      SmartDashboard.putNumberArray("ActualSwerveModuleState", actualLoggingState);

      

    
    updateOdometry();

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
  public void setDesiredStates(ChassisSpeeds cs) {
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
  public void setDesiredStates(ChassisSpeeds cs, Translation2d centerOfRotation) {
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
    
    SmartDashboard.putNumber("FL encoder pos", Math.toDegrees(m_frontLeft.getTurningEncoderRadians()));
    SmartDashboard.putNumber("FR encoder pos", Math.toDegrees(m_frontRight.getTurningEncoderRadians()));
    SmartDashboard.putNumber("BL encoder pos", Math.toDegrees(m_backLeft.getTurningEncoderRadians()));
    SmartDashboard.putNumber("BR encoder pos", Math.toDegrees(m_backRight.getTurningEncoderRadians())); 

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

    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());
    SmartDashboard.putNumber("Robot Angle", m_gyro.getAngle());

  }
}