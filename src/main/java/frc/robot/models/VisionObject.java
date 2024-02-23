package frc.robot.models;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;
// import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class VisionObject {
    public String objectLabel;
    /**
     * One of the following:
     * tag[family]: ID number (ex. "tag16h5: 28")
     * cone
     * cube
     */
    public double x;
    public double y;
    public double z;
    public double r;
    public double xa;
    public double ya;
    public double confidence;

    public DrivetrainSubsystem m_drivetrainSubsystem;
    

    public VisionObject(String objectType, double x, double y, double z, double r, double xa, double ya) {
        this.objectLabel = objectType;
        this.x = x;
        this.y = y;
        this.z = z;
        this.r = r;
        this.xa = xa;
        this.ya = ya;
        confidence = -1;

        m_drivetrainSubsystem = new DrivetrainSubsystem();
    }

    public void motionCompensate(DrivetrainSubsystem drivetrainSubsystem, boolean compensateTranslation) {
        Constants.OBJECT_DETECTION_LATENCY = SmartDashboard.getNumber("Object detection latency", 0.217);
        /*
        if (compensateTranslation) {
        // Normally, we'd subtract the distance travelled.  However, the camera points off the back
        // of the robot.  Therefore, motion in the direction the camera is aiming is returned by
        // getVelocityX() as negative.
            z += drivetrainSubsystem.getKinematicVelocity().x * RobotMap.OBJECT_DETECTION_LATENCY;
            x -= drivetrainSubsystem.getKinematicVelocity().y * RobotMap.OBJECT_DETECTION_LATENCY;
            SmartDashboard.putNumber("X Velocity", drivetrainSubsystem.getKinematicVelocity().x);
            SmartDashboard.putNumber("Z Velocity", drivetrainSubsystem.getKinematicPosition().y);
        }
        */
        
        // double omega = drivetrainSubsystem.getGyroscope().getRate();
        // double theta = omega * RobotMap.OBJECT_DETECTION_LATENCY;

        double theta = 0;
        double targetTime = Timer.getFPGATimestamp() - Constants.OBJECT_DETECTION_LATENCY; // seconds

        for (int i = 0; i < Robot.circularBufferSize; i++) {
            int indexOfInterest = (Robot.bufferSlotNumber + Robot.circularBufferSize - i) % Robot.circularBufferSize; 

            if (Robot.time[indexOfInterest] < targetTime) {
                theta = Units.degreesToRadians(m_drivetrainSubsystem.getGyroscope().getAngle()) - Robot.angle[Robot.bufferSlotNumber] ;// TODO: Turn angle to radiance
                break;
            }
        }
        
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);

        double newZ = z * cosTheta - x * sinTheta;
        double newX = z * sinTheta + x * cosTheta;

        z = newZ;
        x = newX;
    }

    /** Return ID of AprilTag on field */
    public int getAprilTagID() {
        return Integer.valueOf(objectLabel.substring(objectLabel.indexOf(" ") + 1, objectLabel.length()));
    }
    
    /** Returns FIELD centric coordinates of the Robot*/
    public Translation2d getFieldXY() {
        Translation2d FieldXY = new Translation2d(x,z);
        return FieldXY;
    }

    /** Returns robot's heading (angle bot needs to rotate to to be pointing at the apriltag) */
    public Rotation2d getHeadingOffset() {
        Rotation2d HeadingOffset = new Rotation2d(x,z);
        return HeadingOffset;
    }

    public String toString() {
        return "" + x + '\n' + y + '\n' + z + '\n' + r + '\n' + confidence;
    }
}




/*
from marshmallow import Schema, fields


class VisionObject(Schema):
};


/*
from marshmallow import Schema, fields


class VisionObject(Schema):
    objectType = fields.Str(required=True)
    x = fields.Float(required=True)
    y = fields.Float(required=True)
    z = fields.Float(required=True)

class Something(Schema):
    items = fields.List(fields.Nested(VisionObject))
*/