// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SwerveControllerCommand2635;

public class DrivetrainSubsystem extends SubsystemBase {

    private LinkedList<Double> pitchValues = new LinkedList<>();
    private LinkedList<Double> rollValues = new LinkedList<>();
    BuiltInAccelerometer rioAccelerometer = new BuiltInAccelerometer();
    Timer timer = new Timer();


    double smoothedPitch = 0;
    double smoothedRoll = 0;
    double pitchOffset = 0;
    double rollOffset = 0;
    boolean tipCorrection = true;
    boolean PENDING_STATE = false;




    public boolean stopPureVisionAuto = false;

    public static final double kMaxSpeed = 3.63; // 3.63 meters per second  Max Speed for Front, Back, Left, Right
    public final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second   Max Speed for Rotation
    private SwerveModuleState[] swerveModuleStates;

    public static Joystick rightJoystick = RobotContainer.rightJoystick;
    public static Joystick leftJoystick = RobotContainer.leftJoystick;

    // public static Trigger customCenterControlButton = new JoystickButton(leftJoystick, 4);
    

    public final double m_drivetrainWheelbaseWidth =  Constants.DRIVETRAIN_WHEELBASE_WIDTH;  //Calibrated for 2024 BunnyBots
    public final double m_drivetrainWheelbaseLength = Constants.DRIVETRAIN_WHEELBASE_LENGTH; //Calibrated for 2024 BunnyBots

    private double m_angleCache = 180; // This is used to stash the angle before reset, in degrees

    // x is forward       robot is long in the x-direction, i.e. wheelbase length
    // y is to the left   robot is short in the y-direction, i.e. wheelbase width
    // robot front as currently labled on the motors (requires -x trajectory to go out into the +x field direction)
    public final Translation2d m_frontLeftLocation = 
            new Translation2d(-m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);
    public final Translation2d m_frontRightLocation = 
            new Translation2d(m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);
    public final Translation2d m_backLeftLocation = 
            new Translation2d(-m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);
    public final Translation2d m_backRightLocation = 
            new Translation2d(m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);

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
  
    public final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, 200); //new AHRS(SPI.Port.kMXP, (byte) 200);  //Nav X

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation,
      m_frontRightLocation, 
      m_backLeftLocation, 
      m_backRightLocation);

    private boolean followJoystics = true;  // When false does not use Joysticks for driving - When true uses Joysticks for driving
  
    // TODO: if we are going to use path planner, we will need to make the SwerveDriveOdometry() object with the
    //       initialPose parameter.  Not urgent now, but someone should put this into an issue.
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
    getPose(); 

    // resetAngle() should be called before zeroOdometry() because reseting odometry uses gyro values to do the reset
    resetAngle();
    zeroOdometry();
    
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    AutoBuilder.configure(
            // this::getPosePathPlanner, // Robot pose supplier
            this::getPose,
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setDesiredStates(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // Default path replanning config. See the API for the options here
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                return false;
                // var alliance = DriverStation.getAlliance();
                // if (alliance.isPresent()) {
                //   return alliance.get() == DriverStation.Alliance.Red;
                // }
                // return false;
            },
            this // Reference to this subsystem to set requirements
  );
}

  public void stopMotors(){   //Zero motorPower
    m_backLeft.stop();
    m_frontLeft.stop();
    m_backRight.stop();
    m_frontRight.stop();
  }

  public void stashAngle(){
    // m_angleCache = getPose().getRotation().getDegrees();
    m_angleCache = m_gyro.getAngle();
  }

  public void resetAngleAndOdometry(){
    resetAngle();
    zeroOdometry();
  }
  public void restoreAngle(){
    // resetAngle(((m_angleCache + getPose().getRotation().getDegrees() + 180 ) % 360) - 180);
    resetAngle((m_angleCache + m_gyro.getAngle()) % 360);
    // resetOdometry(new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(((m_angleCache + getPose().getRotation().getDegrees() + 180 ) % 360) - 180))));
  }

  // We previously had this toRedHead() in here for converting heading for auto usage
  // now that we have fixed the swerve modules, maybe this is not required... also... is our 
  // 180 deg offset for heading required as well?
  public double toRedHead(double blueHeadingDegrees) {  //Turn Angle from Blue to Red Alliance
    return -1.*(blueHeadingDegrees + 180.);
  }
  public int toRedHead(int blueHeadingDegrees) {
      return -1*(blueHeadingDegrees + 180);
  }

  public Command createPath(Pose2d startPose, Translation2d middlePose, Pose2d endPose){
    int desiredRot = 0; //End Rotation is Defaulted to 0
    return createPath(startPose, middlePose, endPose, desiredRot); 
  }

  public Command createVisionPath(Pose2d startPose, Translation2d middlePose, Pose2d endPose, double endRot){
    return createPath(startPose, middlePose, endPose, endRot, false);
  }
  public Command createVisionPath(Pose2d startPose, Translation2d middlePose, Pose2d endPose, double endRot, boolean centerOfRotationCamera){
    return createPath(startPose, middlePose, endPose, endRot, false, centerOfRotationCamera);
  }

  // Use for open loop paths which needs to be mirrored due to the alliance reflection
  public Command createPath(Pose2d startPose, Translation2d middlePose, Pose2d endPose, double endRot){
    boolean isRedAlliance = false;
    isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    return createPath(startPose, middlePose, endPose, endRot, isRedAlliance);
  }

  public Command createPath(Pose2d startPose, Translation2d middlePose, Pose2d endPose, double endRot, boolean mirrorX){
    return createPath(startPose, middlePose, endPose, endRot, mirrorX, false);
  }

  public Command createPath(Pose2d startPose, Translation2d middlePose, Pose2d endPose, double endRot, boolean mirrorX, boolean centerOfRotationCamera){

    SmartDashboard.putBoolean("mirrorX",mirrorX);
    SmartDashboard.putString("DriverStation.getAlliance().get()",DriverStation.getAlliance().get().name());
    SmartDashboard.putString("DriverStation.Alliance.Red",DriverStation.Alliance.Red.name());

    if (mirrorX) { 
      //Default is Blue so if Red Alliance then negate the X value and Rotation converts to Red Alliance
      startPose = new Pose2d(-startPose.getX(), startPose.getY(), new Rotation2d(Math.toRadians(toRedHead(startPose.getRotation().getDegrees())))); 
      middlePose = new Translation2d(-middlePose.getX(), middlePose.getY());
      endPose = new Pose2d(-endPose.getX(), endPose.getY(), new Rotation2d(Math.toRadians(toRedHead(endPose.getRotation().getDegrees()))));
      endRot *= -1;
    }
    // angleSupplier expects a final variable so we create desiredRot and give the value of endRot
    final double desiredRot = endRot;


    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      Constants.maxModuleLinearSpeed,       // 3.5 m/s
      Constants.maxModuleLinearAccelaration // 4 m/s^2

    ).setKinematics(m_kinematics).setReversed(true);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      startPose,
      List.of(
        middlePose
      ),
      endPose,
      trajectoryConfig
    );

    TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Constants.kMaxModuleAngularSpeedRadiansPerSecond, Constants.kMaxModuleAngularAccelerationRadiansPerSecondSquared);
    // This x, y, and theta controllers are the controllers which are used for feedback inside 
    // the holonomic drive controller created by the SwerveControllerCommand.  These controllers 
    // close the loop around m_poseError and m_rotationError.  These caluclations are performed in 
    // HolonomicDriveController.calculate().  Note that while they call it thetaFF in .calcualate()
    // covers the error in heading and is controlled by the thetaController we create here.
    //
    // Important Note... we can disable the feedback and closing the loop around the x,y positions 
    // in the trajectory by setting kp to 0 for the x and y controllers and only feed forward velocity
    // commands will be set during the trajectory.  So for tuning the path, we can run a bunch of 
    // trajectory commands (preferibly not auto so we can do multiple commands sequentially... would need
    // to ensure we reset the odometry etc at the beginning of the sequetial command), then we can
    // slowly increase kp up to the point that we get the desired performance.  Note that if we have 
    // overshoot in the path, then we can increase kd.
    //
    // Note that for debugging, it would be good to implement our own SwerveControllerCommand.java
    // class for our own use.  This would then override the execute() function and insert data into  
    // the data logger / shuffleboard so we can directly compare the desired states , actual states
    // target chassisSpeed and actual chassis speed.  Something like this...
    // @Override
    // public void execute() {
    //   super.execute()
    //   // instrumentation for shuffleboard logging goes here.
    // }
    PIDController xController = new PIDController(6, 6, 1);
    PIDController yController = new PIDController(6, 6, 1);
    // kp = 0.4, ki = 3.3, kd = 1 integral overshot
    // Note: We reduced Kp to 2 so that rottion control loop doesn't saturate the module motor speed during autos
    // This however makes it so that robot cannot turn quickly, which is not good however it enables more acurate and consistent auto paths
    ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0.2, kThetaControllerConstraints); // Find a value for the PID
    // ProfiledPIDController thetaController = new ProfiledPIDController(4, 0, 0.2, kThetaControllerConstraints); // Find a value for the PID
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //This SwerveControllerCommand expects an angleSupplier
    Supplier<Rotation2d> angleSupplier = () -> (Rotation2d)(Rotation2d.fromDegrees(desiredRot));

    // the SwerveControllerCommand makes a HolonomicDriveController() out of the x, y, theta
    // controllers and inside HolonomicDriveController constructor, enables the continuous input 
    // on the theta controller from 0 to 360.  Does this create problems if we try to input -45 deg
    // as a target heading?

    SwerveControllerCommand2635 swerveControllerCommand = new SwerveControllerCommand2635(
      trajectory,
      this::getPose,
      m_kinematics,
      xController,
      yController,
      thetaController,
      angleSupplier,
      this::setModuleStates,  // This is a consumer to set the states as defined in docs for SwerveControllerCommand
      this
    );

    return swerveControllerCommand;
  }

  public void resetAngle(){
    // Setting the angle adjustment changes where forward is when you push the controls forward
    // However it doesn't rotate the definition of the odometry x and y
    resetAngle(0);
  }
  // do we use resetAngle(degree) when starting the auto from some angle which is not aligned with 
  // the front of the robot pointing downfield?
  public void resetAngle(double degree){
    //Use this method if you want to reset the angle to something not 0
    m_gyro.reset();
    m_gyro.setAngleAdjustment(degree);
  }

  public void setFollowJoystick(boolean followJoystics){
    this.followJoystics = followJoystics;
  }

  public void setStopVisionAutoCommand(boolean pureVisionAutoState){
    stopPureVisionAuto = pureVisionAutoState;
  }
  
  public boolean getStopVisionAutoCommand(){
    return stopPureVisionAuto;
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
    // Input a value between -1 and 1 for angular velocity of robot, it is later multiplied by `kMaxAngularSpeed` in drive

    rotCommanded = rot;
  }

  private void addPitchValue(double newValue) {
    if (pitchValues.size() >= Constants.WINDOW_SIZE) {
        pitchValues.removeFirst();
    }
    pitchValues.add(newValue);
  }

  private void addRollValue(double newValue) {
    if (rollValues.size() >= Constants.WINDOW_SIZE) {
      rollValues.removeFirst();
    }
    pitchValues.add(newValue);
  }

  public void setDriveSpeed(double time){
    timer.start();
    setFollowJoystick(false);
    while(timer.get() < time){
      drive(2.5, 0, 0, true);
    }
    setFollowJoystick(true);
    timer.stop();
    timer.reset();
  }

  public double getYawGyroValue(){
    return m_gyro.getRawGyroZ();
  }

  private double calculateSmoothedValue(LinkedList<Double> values) {
    double smoothedValue = 0.0;
    double weight = 1.0;
    double totalWeight = 0.0;

    for (double value : values) {
        smoothedValue += value * weight;
        totalWeight += weight;
        weight *= Constants.SMOOTHING_FACTOR;
    }

    return smoothedValue / totalWeight;
  }

  public void setAntiTipOffsets(){
    pitchOffset = m_gyro.getPitch();
    rollOffset = m_gyro.getRoll();
  }

  public void setTriggerAntiTip(boolean pending){
    PENDING_STATE = pending;
  }

  public void setEnableAntiTip(){
    tipCorrection = PENDING_STATE;
  }

  public void setAntiTip(boolean bool){
    tipCorrection = bool;
  }

  public ChassisSpeeds getAntiTipCorrections(){
    if(tipCorrection && isTipping()){
      double pitch = m_gyro.getPitch()-pitchOffset;
      double roll = m_gyro.getRoll()-rollOffset;
      double pitchCorrection = 0;
      double rollCorrection = 0;
      //Multiplying by NOSE_DOWN_PITCH and RIGHT_ROLL to capture directionality gyro coordinates versus robot coordinates
      if(pitch == Math.abs(pitch) * Constants.NOSE_DOWN_PITCH){
        pitchCorrection = pitch * Constants.NOSE_DOWN_PITCH * Constants.PITCH_NOSE_DOWN_PROPORTION_CONSTANT * kMaxSpeed; 
      }
      else{
        pitchCorrection = pitch * Constants.NOSE_DOWN_PITCH * Constants.PITCH_NOSE_UP_PROPORTION_CONSTANT * kMaxSpeed;
      }

      if(roll == Math.abs(roll) * Constants.RIGHT_ROLL){
        rollCorrection = roll * Constants.RIGHT_ROLL * Constants.ROLL_RIGHT_PROPORTION_CONSTANT * kMaxSpeed; 
      }
      else{
        rollCorrection = roll * Constants.RIGHT_ROLL * Constants.ROLL_LEFT_PROPORTION_CONSTANT * kMaxSpeed;
      }
      SmartDashboard.putNumber("pitchCorrection", pitchCorrection);
      SmartDashboard.putNumber("rollCorrection", rollCorrection);
      return new ChassisSpeeds(rollCorrection, pitchCorrection, 0);
    }
    return new ChassisSpeeds(0, 0, 0);
  }

    
  public boolean isTipping() {
    double pitch = m_gyro.getPitch() - pitchOffset;
    double roll = m_gyro.getRoll() - rollOffset;
    
    SmartDashboard.putNumber("pitchWithOffset", pitch);
    SmartDashboard.putNumber("rollWithOffset", roll);


    // addRollValue(roll);
    // addPitchValue(pitch);

    // smoothedPitch = calculateSmoothedValue(pitchValues);
    // smoothedRoll = calculateSmoothedValue(rollValues);

    return Math.abs(pitch) > Constants.TIPPING_ANGLE_THRESHOLD || Math.abs(roll) > Constants.TIPPING_ANGLE_THRESHOLD;
  }
  



  @Override
  public void periodic() {
    
    //Hat Power Overides for Trimming Position and Rotation
    // System.out.println("X: "+getPose().getX()+"\tY: "+getPose().getY()+"\tRot: "+getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("gyro.pitch()", m_gyro.getPitch());
    SmartDashboard.putNumber("gyro.roll()", m_gyro.getRoll());
    SmartDashboard.putBoolean("isTipping", isTipping());

    SmartDashboard.putNumber("stashAngle", m_angleCache);
    SmartDashboard.putNumber("BackRight turn", m_backRight.getTurningEncoderRadians());
    SmartDashboard.putNumber("BackLeft turn", m_backLeft.getTurningEncoderRadians());
    SmartDashboard.putNumber("FrontRight turn", m_frontRight.getTurningEncoderRadians());
    SmartDashboard.putNumber("FrontLeft turn", m_frontLeft.getTurningEncoderRadians());

    SmartDashboard.putNumber("BackRight pos", m_backRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("BackLeft pos", m_backLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("FrontRight pos", m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("FrontLeft pos", m_frontLeft.getPosition().distanceMeters);
    // To stop the wheels from moving when there is no input from the joysticks
    xPowerCommanded =0;
    yPowerCommanded =0;
    rotCommanded =0;
    
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
        rotCommanded = Constants.HAT_POWER_ROTATE*-1;
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

      // TODO: look at the deadband below
      if (Math.pow(rightJoystick.getTwist(),3)>0.05 || Math.pow(rightJoystick.getTwist(),3)<-0.05) {
        rotCommanded = rightJoystick.getTwist()*-1;
      }

      // TODO: document how to use this button to reset various robot centers of rotation
      // Note: you can have multiple buttons for defining multiple centers of rotation.
      // if (customCenterControlButton.getAsBoolean()) {
      //   this.drive(-xPowerCommanded * DrivetrainSubsystem.kMaxSpeed, 
      //           yPowerCommanded * DrivetrainSubsystem.kMaxSpeed,
      //           MathUtil.applyDeadband(-rotCommanded * this.kMaxAngularSpeed, 0.2), 
      //           true,
      //           new Translation2d(0, -Constants.DRIVETRAIN_WHEELBASE_LENGTH/2));
      // } else {
      this.drive(
              xPowerCommanded * DrivetrainSubsystem.kMaxSpeed, 
              yPowerCommanded * DrivetrainSubsystem.kMaxSpeed,
              MathUtil.applyDeadband(rotCommanded * this.kMaxAngularSpeed, 0.2), 
              true);
      }
    // }
    SmartDashboard.putNumber("m_gyro.getRawGyroZ", getYawGyroValue());
    SmartDashboard.putNumber("FL_pos", m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("FR_pos", m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("BL_pos", m_backLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("BR_pos", m_backRight.getPosition().distanceMeters);
    
    SmartDashboard.putNumber("rotCommanded", rotCommanded);

    double loggingStateForAdvantageScope[] = {     //Array for predicted values
      swerveModuleStates[Constants.FRONT_LEFT_MODULE_STATE_INDEX].angle.getDegrees() - 90, // Order here is BR, FR, BL, FL; order on Advantage Scope is FL, FR, BL, BR, but it works like this and we don't know why
      swerveModuleStates[Constants.FRONT_LEFT_MODULE_STATE_INDEX].speedMetersPerSecond,
      swerveModuleStates[Constants.FRONT_RIGHT_MODULE_STATE_INDEX].angle.getDegrees() - 90,
      swerveModuleStates[Constants.FRONT_RIGHT_MODULE_STATE_INDEX].speedMetersPerSecond,
      swerveModuleStates[Constants.BACK_LEFT_MODULE_STATE_INDEX].angle.getDegrees() - 90,
      swerveModuleStates[Constants.BACK_LEFT_MODULE_STATE_INDEX].speedMetersPerSecond,
      swerveModuleStates[Constants.BACK_RIGHT_MODULE_STATE_INDEX].angle.getDegrees() - 90,
      swerveModuleStates[Constants.BACK_RIGHT_MODULE_STATE_INDEX].speedMetersPerSecond,
    };

    double loggingActualStateForAdvantageScope[] = {
      (m_frontLeft.getTurningEncoderRadians() * 180 / Math.PI) - 90, // same order problem as predicted values
      m_frontLeft.getVelocity(),
      (m_frontRight.getTurningEncoderRadians() * 180 / Math.PI) - 90,
      m_frontRight.getVelocity(),
      (m_backLeft.getTurningEncoderRadians() * 180 / Math.PI) - 90,
      m_backLeft.getVelocity(),
      (m_backRight.getTurningEncoderRadians() * 180 / Math.PI) - 90,
      m_backRight.getVelocity(),
    };

    SmartDashboard.putNumberArray("loggingStateForAdvantageScope",loggingStateForAdvantageScope);
    SmartDashboard.putNumberArray("loggingActualStateForAdvantageScope", loggingActualStateForAdvantageScope);


    updateOdometry();

    putDTSToSmartDashboard();
    tuneAngleOffsetPutToDTS();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).   -1.0 ... +1.0
   * @param ySpeed Speed of the robot in the y direction (sideways).  -1.0 ... +1.0
   * @param rot Angular rate of the robot.                            -1.0 ... +1.0
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * This function is based off of the center of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, fieldRelative, new Translation2d(0, 0));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).   -1.0 ... +1.0
   * @param ySpeed Speed of the robot in the y direction (sideways).  -1.0 ... +1.0
   * @param rot Angular rate of the robot.                            -1.0 ... +1.0
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param centerOffset is offset from center of robot to custom center of rotation in meters.
   * * left is positive x, front is positive y.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d centerOffset) {
    // TODO: Move kMaxSpeed and kMaxRotation into this method for ySpeed and xSpeed, and rot
    // TODO: Add another parameter for kMaxSpeed so you have an option to set it
    ChassisSpeeds chassisSpeeds = fieldRelative
    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
    : new ChassisSpeeds(xSpeed, ySpeed, rot);
    chassisSpeeds = chassisSpeeds.plus(getAntiTipCorrections());

    swerveModuleStates = 
        m_kinematics.toSwerveModuleStates(
            chassisSpeeds,
            centerOffset
        );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    //If the desired states are not in this order then the swerve will not work  
    m_frontLeft.setDesiredState(swerveModuleStates[Constants.FRONT_LEFT_MODULE_STATE_INDEX]);
    m_frontRight.setDesiredState(swerveModuleStates[Constants.FRONT_RIGHT_MODULE_STATE_INDEX]);
    m_backLeft.setDesiredState(swerveModuleStates[Constants.BACK_LEFT_MODULE_STATE_INDEX]);
    m_backRight.setDesiredState(swerveModuleStates[Constants.BACK_RIGHT_MODULE_STATE_INDEX]);

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
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

  public Pose2d getPosePathPlanner(){
    Pose2d rawPose = m_odometry.getPoseMeters();
    
    Translation2d adjTranslation = rawPose.getTranslation().rotateBy(Rotation2d.fromDegrees(90));
    Rotation2d adjRotation = rawPose.getRotation().plus(Rotation2d.fromDegrees(90));
    
    return new Pose2d(adjTranslation, adjRotation);
  }
  // public Pose2d getPoseMeters() {
  //   Pose2d currentPose = getPose();
  //   return new Pose2d(currentPose.getTranslation().div(39.37), currentPose.getRotation());
  // }

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
  
    m_frontLeft.setDesiredState(desiredStates[Constants.FRONT_LEFT_MODULE_STATE_INDEX]);
    m_frontRight.setDesiredState(desiredStates[Constants.FRONT_RIGHT_MODULE_STATE_INDEX]);
    m_backLeft.setDesiredState(desiredStates[Constants.BACK_LEFT_MODULE_STATE_INDEX]);
    m_backRight.setDesiredState(desiredStates[Constants.BACK_RIGHT_MODULE_STATE_INDEX]);
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
    
    m_frontLeft.setDesiredState(desiredStates[Constants.FRONT_LEFT_MODULE_STATE_INDEX]);
    m_frontRight.setDesiredState(desiredStates[Constants.FRONT_RIGHT_MODULE_STATE_INDEX]);
    m_backLeft.setDesiredState(desiredStates[Constants.BACK_LEFT_MODULE_STATE_INDEX]);
    m_backRight.setDesiredState(desiredStates[Constants.BACK_RIGHT_MODULE_STATE_INDEX]);
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
    m_frontLeft.setDesiredState(desiredStates[Constants.FRONT_LEFT_MODULE_STATE_INDEX]);
    m_frontRight.setDesiredState(desiredStates[Constants.FRONT_RIGHT_MODULE_STATE_INDEX]);
    m_backLeft.setDesiredState(desiredStates[Constants.BACK_LEFT_MODULE_STATE_INDEX]);
    m_backRight.setDesiredState(desiredStates[Constants.BACK_RIGHT_MODULE_STATE_INDEX]);
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

    SmartDashboard.putNumber("getPose.getX", getPose().getX());
    SmartDashboard.putNumber("getPose.getY", getPose().getY());
    SmartDashboard.putNumber("gyro.getAngle", m_gyro.getAngle());
    SmartDashboard.putNumber("getPose.getRotation", getPose().getRotation().getDegrees());

    SmartDashboard.putBoolean("tipCorrection", tipCorrection);
  }
}