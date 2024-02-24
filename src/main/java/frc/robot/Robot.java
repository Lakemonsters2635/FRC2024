// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;  

  private RobotContainer m_robotContainer;

  // private DoubleSubscriber xSub;
  // private DoubleSubscriber ySub;
  // private DoubleSubscriber zSub;
  // private GenericSubscriber objectSub;

  // private DoubleSubscriber xSubBack;
  // private DoubleSubscriber ySubBack;
  // private DoubleSubscriber zSubBack;
  // private GenericSubscriber objectSubBack;

  // public static double x;
  // public double y;
  // public static double z;
  // public String object;

  // public double xBack;
  // public double yBack;
  // public double zBack;
  // public String objectBack;

  // for motion compensate (vision)
  public static int circularBufferSize = 50;
  public static int bufferSlotNumber = 0;
  public static double[] time;
  public static double[] angle;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // motion compensate (vision)
    time = new double[circularBufferSize]; 
    angle =  new double[circularBufferSize];

    // Front camera network table
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("MonsterVision");
    // xSub = table.getDoubleTopic("x").subscribe(0);
    // ySub = table.getDoubleTopic("y").subscribe(0);
    // zSub = table.getDoubleTopic("z").subscribe(0);
    // objectSub = table.getStringTopic("objectLabel").genericSubscribe("");

    // // Back camera network table
    // NetworkTable tableBack = NetworkTableInstance.getDefault().getTable("");// TODO: Enter back camera key
    // xSubBack = tableBack.getDoubleTopic("x").subscribe(0);
    // ySubBack = tableBack.getDoubleTopic("y").subscribe(0);
    // zSubBack = tableBack.getDoubleTopic("z").subscribe(0);
    // objectSubBack = tableBack.getStringTopic("object").genericSubscribe("");

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // m_autoChooser = m_robotContainer.getAutonomousCommand();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // RobotContainer.m_armSubsystem.putToBoard();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // m_robotContainer.m_objectTrackerSubsystemAprilTagPro.data();
    // m_robotContainer.m_objectTrackerSubsystemNoteCam.data();
    // SmartDashboard.putString("NetworkTables Note Cam", m_robotContainer.m_objectTrackerSubsystemNoteCam.getObjectsJson());
    // SmartDashboard.putString("NetworkTables April Tag Pro", m_robotContainer.m_objectTrackerSubsystemAprilTagPro.getObjectsJson());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_autoChooser.getSelected();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // RobotContainer.m_armSubsystem.setPosTarget(RobotContainer.m_armSubsystem.getTheta());
    // System.out.println("Theta value: "+RobotContainer.m_armSubsystem.getTheta());
    RobotContainer.m_drivetrainSubsystem.zeroOdometry();
    RobotContainer.m_drivetrainSubsystem.resetAngle();
    // RobotContainer.m_armSubsystem.m_poseTarget2=80;
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RobotContainer.m_driveTrainCommand.execute();
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}