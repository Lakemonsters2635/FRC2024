// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class VisionAutoCommand extends Command {
  /** Creates a new VisionAutoCommand. */
  DrivetrainSubsystem m_dts;
  ObjectTrackerSubsystem m_ots;
  double visionX;
  double visionY;
  double visionZ;

  double fieldX;
  double fieldY;
  public VisionAutoCommand(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots) {
    m_dts = dts;
    m_ots = ots;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Don't need to get m_ots.data() because it is already called in Robot.java periodic

    try{
    visionX = m_ots.visionX;
    visionY = m_ots.visionY;
    visionZ = m_ots.visionZ;

    var detectionObject = m_ots.getNearestAprilTagDetection();

    double radius = m_ots.getRadius(detectionObject);
    double thetaYZ = m_ots.getThetaYZField(detectionObject);

    SmartDashboard.putNumber("Robot x", m_dts.getPose().getX());
    SmartDashboard.putNumber("Robot y", m_dts.getPose().getY());
    SmartDashboard.putNumber("Robot rot", m_dts.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("NAVX angle", m_dts.m_gyro.getAngle());
    System.out.println("AHRS_angle" + m_dts.m_gyro.getAngle());

    SmartDashboard.putNumber("radius", radius);
    SmartDashboard.putNumber("thetaYZ", thetaYZ);

    }
    catch(Exception e) {
      // System.out.println(e);
    }
    visionCreatePath().schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dts.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  public Command visionCreatePath(){
    // m_ots.data();
    // visionX = m_ots.visionX;
    // visionZ = m_ots.visionZ;
    // visionY = m_ots.visionY;
    SmartDashboard.putNumber("visionXAuto", visionX);
    SmartDashboard.putNumber("visionYAuto", visionY);
    SmartDashboard.putNumber("visionZAuto", visionZ);

    Pose2d botPose = m_dts.getPose();

    
    // SmartDashboard.putNumber("deltaFieldX", deltaFieldX);
    // SmartDashboard.putNumber("deltaFieldY", deltaFieldY);

    double deltaRobotX = -Units.inchesToMeters(visionX); // We are facing the april tag first so there is no need to change in robot x
    double deltaRobotY = -Units.inchesToMeters(visionZ)+1; // We want to end our auto 1 meter away from the apriltag

    double botRadians = botPose.getRotation().getRadians();
    
    // Figure out the trigonometri which converts deltaRobotX and deltaRobotY to deltaFieldX and deltaFieldY
    double deltaFieldX = (deltaRobotX*Math.cos(botRadians))+ (deltaRobotY*Math.sin(botRadians));
    double deltaFieldY = -(deltaRobotX*Math.sin(botRadians))+ (deltaRobotY*Math.cos(botRadians));

    deltaFieldX *=-1;
    // deltaFieldY += 1 + Units.inchesToMeters(13.5);

    SmartDashboard.putNumber("deltaRobotX", deltaRobotX);
    SmartDashboard.putNumber("deltaRobotY", deltaRobotY);
    SmartDashboard.putNumber("deltaFieldX", deltaFieldX);
    SmartDashboard.putNumber("deltaFieldY", deltaFieldY);

    
    int i = 0;
    return new SequentialCommandGroup(
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() x before",m_dts.getPose().getX())),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() y before",m_dts.getPose().getY())),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() rotation before",m_dts.getPose().getRotation().getDegrees())),
      m_dts.createVisionPath(
        new Pose2d(
          botPose.getX(), 
          botPose.getY(), 
          new Rotation2d(botPose.getRotation().getRadians()-(Math.PI/2))   // TODO need to explain this rotation offset and point to docs
        ), 
        new Translation2d(
          botPose.getX()+(deltaFieldX/2), 
          botPose.getY()+(deltaFieldY/2)
        ), 
        new Pose2d(
          botPose.getX()+deltaFieldX,
          botPose.getY()+deltaFieldY, 
          new Rotation2d(botPose.getRotation().getRadians()-(Math.PI/2))
        ),
        botPose.getRotation().getDegrees()
      ),
      new InstantCommand(()->m_dts.stopMotors()),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() x after",m_dts.getPose().getX())),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() y after",m_dts.getPose().getY())),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() rotation after",m_dts.getPose().getRotation().getDegrees()))
    );
    // return m_dts.createPath(
    //   new Pose2d(botPose.getX(), botPose.getY(), botPose.getRotation()), 
    //   new Translation2d(botPose.getX()+(Units.inchesToMeters(visionX)/2), botPose.getY()+(Units.inchesToMeters(visionY)/2)), 
    //   new Pose2d(botPose.getX()+Units.inchesToMeters(visionX), botPose.getY()+Units.inchesToMeters(visionY)-1, new Rotation2d(botPose.getRotation().getRadians()+ Math.atan(visionX/visionZ))));
  }
}
