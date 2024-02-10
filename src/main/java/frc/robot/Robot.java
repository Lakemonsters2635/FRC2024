// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private SendableChooser<Command> m_autoChooser;

  private RobotContainer m_robotContainer;

  private DoubleSubscriber xSub;
  private DoubleSubscriber ySub;
  private DoubleSubscriber zSub;
  private GenericSubscriber objectSub;

  private DoubleSubscriber xSubBack;
  private DoubleSubscriber ySubBack;
  private DoubleSubscriber zSubBack;
  private GenericSubscriber objectSubBack;

  public double x;
  public double y;
  public double z;
  public String object;

  public double xBack;
  public double yBack;
  public double zBack;
  public String objectBack;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Front camera network table
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Monster Vision");
    xSub = table.getDoubleTopic("x").subscribe(0);
    ySub = table.getDoubleTopic("y").subscribe(0);
    zSub = table.getDoubleTopic("z").subscribe(0);
    objectSub = table.getStringTopic("object").genericSubscribe("");

    // Back camera network table
    NetworkTable tableBack = NetworkTableInstance.getDefault().getTable("");// TODO: Enter back camera key
    xSubBack = tableBack.getDoubleTopic("x").subscribe(0);
    ySubBack = tableBack.getDoubleTopic("y").subscribe(0);
    zSubBack = tableBack.getDoubleTopic("z").subscribe(0);
    objectSubBack = tableBack.getStringTopic("object").genericSubscribe("");

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_autoChooser = m_robotContainer.getAutonomousCommand();
    SmartDashboard.putData("AutoChooser",m_autoChooser);

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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_autoChooser.getSelected();

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
    RobotContainer.m_drivetrainSubsystem.zeroOdometry();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Front camera values
    x = xSub.get();
    y = ySub.get();
    z = zSub.get();
    object = objectSub.getString("");

    // Back camera values
    xBack = xSubBack.get();
    yBack = ySubBack.get();
    zBack = zSubBack.get();
    objectBack = objectSubBack.getString("");
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