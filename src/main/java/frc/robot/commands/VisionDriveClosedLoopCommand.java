package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.models.VisionObject;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

// import com.kauailabs.navx.frc.AHRS;  
// import edu.wpi.first.wpilibj.SPI;

/**
 * Closed-loop drive to cone, cube, or AprilTag 
  Copy of FetchPowerCellCommand with modified contructor to take cargo color
*/

public class VisionDriveClosedLoopCommand extends CommandBase {
  DrivetrainSubsystem m_drivetrainSubsystem;
  ObjectTrackerSubsystem m_objectTrackerSubsystem;

  PIDController angleController;
  PIDController strafeController;
  PIDController forwardController; 
  double gyroAngle;
  double angle; 
  double desiredAngle;
  double setPointAngle = 6;
  boolean isClose;
  boolean lostObject = false;
  boolean m_stopAtEnd = true;       // Do we set the drive speed to zero at the end?
  double m_targetPoseAngle;
  double m_lastDistanceSeen;
  boolean m_wrongObject;
  boolean m_goSlow;                 // This is a true HACK!  For some reason, once we are about to
                                    // reach the cube, we suddenly go back to full speed.
                                    // The purpose of this variable is to make slow speed
                                    // "sticky".  TODO: debug this after Salem.

  String targetObjectLabel; // cone, cube, or AprilTag
  int aprilTagID;
  int targetTagID = -1; // depends on whether you're red/blue and starting on the right/left side of the charge station
  Timer timer = new Timer();
  double now;

  int triggerDistance = 0;

  private boolean inTeleop = false; 

  // TODO: note that the targetObjectLabel here masks the targetObjectLabel, likely should have a different name for 
  // the constructor argument and the instance variable this.targetObjectLabel
  public VisionDriveClosedLoopCommand(String targetObjectLabel, boolean inTeleop, DrivetrainSubsystem ds, ObjectTrackerSubsystem ots, boolean stopAtEnd, int... Id) {
    //initPID();
    checkUpdateObjectLabel(targetObjectLabel); 
    this.inTeleop = inTeleop;    
    m_drivetrainSubsystem = ds;
    m_objectTrackerSubsystem = ots;
    m_stopAtEnd = stopAtEnd;
    aprilTagID = Id.length == 1 ? Id[0] : -1;   // Id parameter overrides value in label

    // TODO: Figure out this part
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      // Source
      if (aprilTagID<=10 && aprilTagID>=9) {
        aprilTagID -=8;
      }
      // Speaker
      else if(aprilTagID>=3 && aprilTagID<=4){
        aprilTagID +=4;
      }
      // Amp
      else if(aprilTagID==5){
        aprilTagID +=1;
      }

      // Old code:
      aprilTagID += 5;
    }
    addRequirements(m_drivetrainSubsystem);
  }

  // TODO: beter name? This actually sets / initializes the targetObjectLabel instance parameter
  private void checkUpdateObjectLabel(String label) {
    // checks targetObjectLabel given to constructor, normalizes case
    // if (label.equalsIgnoreCase("cone")) {
    //   this.targetObjectLabel = "cone";
    //   this.triggerDistance = Constants.TARGET_TRIGGER_DISTANCE_CONE;

    // } else if (label.equalsIgnoreCase("cube")) {
    //   this.targetObjectLabel = "cube"; 
    //   this.triggerDistance = Constants.TARGET_TRIGGER_DISTANCE_CUBE;

    // } else if (label.equalsIgnoreCase("any")) {
    //   this.targetObjectLabel = "any"; 
    //   this.triggerDistance = Constants.TARGET_TRIGGER_DISTANCE_ANY;
    // } 
    if (label.contains("tag32h11")) {
      this.targetObjectLabel = "tag";
      try {
        this.aprilTagID = Integer.valueOf(label.substring(label.indexOf(" "),label.length()-1)); 
      } catch (Exception e) {
        this.aprilTagID = -1;
      }
      this.triggerDistance = Constants.TARGET_TRIGGER_DISTANCE_APRIL_TAG;
    } else {
      // This should never happen.
      // System.out.println("ERROR: UNKNOWN targetObjectLabel in VisionDriveClosedLoopCommand.checkUpdateObjectLabel(). Vision drive closed loop command will not behave as expected.");
    }
  }

  protected void initPID(){
    angleController = new PIDController(0.08, 0.0, 0.0);
    strafeController = new PIDController(0.02, 0.0, 0.0); // TODO update constants
    forwardController = new PIDController(0.05, 0.01, 0.0); // TODO update constants   
  }

  @Override
  public void initialize() {
    initPID();
    // System.out.println("FCC start");
    
    // SmartDashboard.putNumber("Vision angle", angle);
    // SmartDashboard.putNumber("Desired angle", desiredAngle);
    // SmartDashboard.putNumber("initial angle", gyroAngle);
    // SmartDashboard.putNumber("SetPoint angle", setPointAngle);
    
    // LEGACY RESET KINEMATICS
    // Vector2 position = new Vector2(0, 0);
    // m_drivetrainSubsystem.resetKinematics(position, 0);
    m_drivetrainSubsystem.zeroOdometry(); // TODO not sure if we want to do this??

    // System.out.println("Initialized FCC");

    isClose = false;
    m_lastDistanceSeen = 1000;              // Really big....
    m_wrongObject = false;
    m_goSlow = false;

    // Try to align with field.  Make our pose angle 0deg or 180deg; whichever is closest

    m_targetPoseAngle = Math.cos(m_drivetrainSubsystem.m_gyro.getAngle()) > 0 ? 0 : 180;// TODO: added Math.cos not sure whether works or not

    m_objectTrackerSubsystem.data();

    try {
      SmartDashboard.putNumber("FCC closest object z", getObjectOfInterest().z);
    } catch (Exception e) {}
  }

  private VisionObject getObjectOfInterest()
  {
    return targetObjectLabel == "tag" && aprilTagID != -1 ? 
      m_objectTrackerSubsystem.getSpecificAprilTag(aprilTagID) :
      (targetObjectLabel == "any" ? m_objectTrackerSubsystem.getClosestObject() : m_objectTrackerSubsystem.getClosestObject(targetObjectLabel));
  }
 
  @Override
  public void execute() {
    m_objectTrackerSubsystem.data();
    double forward = 0;
    double strafe = 0;
    double rotation = 0;

    VisionObject closestObject = getObjectOfInterest();
     
    if (closestObject == null || closestObject.z > 200) {
      // SmartDashboard.putNumber("driveRotation", 99);
      m_drivetrainSubsystem.drive(0, 0, 0, false);
      // SmartDashboard.putString("FCC Status", "No cargo in frame OR cargo out of range");
      return; // no object found
    }

    // As we approach the object, if it suddenly gets much further away, it is probably a different
    // object.  Since we don't want to accidentally chase an object on the wrong side of the field,
    // treat this case as a lost object

    if (closestObject.z > (m_lastDistanceSeen + 48))
    {
      m_wrongObject = true;
      return;
    }
    
    m_lastDistanceSeen = closestObject.z;

    double v; // velocity? 3/14
    // System.out.println("Closest z: " + closestObject.z);
    // closestObject.motionCompensate(m_drivetrainSubsystem, true);

    rotation = 0;

    if (this.targetObjectLabel == "tag")
    {
      // Get current pose, normalized to 0-360deg

      double poseAngle = m_drivetrainSubsystem.m_gyro.getAngle() % 360; // TODO: not sure works or not deleted .getDegreees() after getAngle()
      if (poseAngle < 0)
        poseAngle += 360;

      // If target pose is 0 and current pose > 270,
      // subtract 360 from current pose such that 270..360 becomes -90..0
      if (m_targetPoseAngle == 0 && poseAngle > 270)
        poseAngle -= 360;

      rotation = -angleController.calculate(poseAngle, m_targetPoseAngle);
    }
    
    if (rotation > 1){
      rotation = 1;
    } else if (rotation < -1) {
      rotation = -1;
    }

    SmartDashboard.putNumber("driveRotation", rotation);
    
    // strafe

    strafeController.setSetpoint(closestObject.x);
    strafe = strafeController.calculate(0);

    if(strafe > 1){
      strafe = 1;
    }else if (strafe < -1){
      strafe = -1;
    }

    // SmartDashboard.putNumber("driveStrafe", strafe);

    // forward
    //forwardController.setSetpoint(closestObject.z-RobotMap.TARGET_TRIGGER_DISTANCE); // TODO figure out how to implement code that begins intake process 
    forward = forwardController.calculate(0);

    if(forward > 1){
      forward = 1;
    }else if (forward < -1){
      forward = -1;
    }

    // SmartDashboard.putNumber("driveForward", forward);
    
    final boolean robotOriented = false;

    //final Vector2 translation = new Vector2(-forward, -strafe*0);
  
    v = -0.4;  

    if (inTeleop) {
      v = -1.2; // -0.7
    }

    if (m_goSlow || (closestObject.z < 30 && this.targetObjectLabel != "tag")) {   // Slow down when close, unless AprilTag
      v = -0.3;
      m_goSlow = true;      // Once we slow down, don't speed up.  TODO: This is a HACK!
    }
    //  if (closestObject.z < 60) {
    //    isClose = true;
    // //   v = -0.05;
     
    //  }
    // final Vector2 translation = new Vector2(v, strafe);
    // System.out.println("translation: " + translation);
    m_drivetrainSubsystem.drive(v, strafe, rotation, robotOriented);
  }

@Override
public boolean isFinished() {
  double tolerance = 4; // TODO units...? i think it's inches
  if (m_wrongObject)
  {
    System.out.println("Object switch");
    return true;
  }
  VisionObject closestObject = getObjectOfInterest();
  if(closestObject == null) {
    if (lostObject)
    {
      now =  timer.get();
      if (now > 0.5)
        System.out.println("Object lost");
      return now > 0.5;
    }
    timer.restart();
    lostObject = true;
    return false;
  }
  
  lostObject = false;
  // boolean done = Math.abs(closestObject.z - Constants.TARGET_TRIGGER_DISTANCE) <= tolerance  }

  isClose = Math.abs(closestObject.z - triggerDistance) <= tolerance;
  // if (done) {
  //   System.out.println("done FCC");
  // }
  if (isClose) {
    System.out.println("FCC done");
  }
  return isClose;
}

  @Override
  public void end(boolean interrupted) {
    // Robot.drivetrainSubsystem.holonomicDrive(new Vector2(-100.0, 0.0), 0, true);
    // System.out.println("FCC end() drive forward extra 5 in");
    if (m_stopAtEnd) {
      m_drivetrainSubsystem.drive(0, 0, 0, true);
      System.out.println("stopping at end of VisionDriveClosedLoopCommand");
    }
    // System.out.println("FCC end()");
  }

}