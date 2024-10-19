package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import com.google.gson.Gson;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.models.VisionObject;

// TODO: Harmonize Detection and VisionObject and refactor code
class Detection{
    public String objectLabel;
    public double x;
    public double y;
    public double z;
    public double confidence;
    public double xa;
    public double ya;
    public double za;
}
class DetectionList extends ArrayList<Detection> {
    @Override
    public boolean add(Detection detec) {
        return super.add(detec);
    }
    @Override
    public Detection get(int index) {
        return super.get(index);
    }
    @Override
    public Detection remove(int index) {
        return super.remove(index);
    }
}

public class ObjectTrackerSubsystem extends SubsystemBase {
	NetworkTable monsterVision; 
    public VisionObject[] foundObjects; 
    private String jsonString;
    private String source;
    private Gson gson = new Gson();
    public double visionZ;
    public double visionX;
    public double visionY;


    /*
     * Red Alliance Community (right to left) – IDs 1, 2, 3
     * Blue Alliance Double Substation – ID 4
     * Red Alliance Double Substation – ID 5
     * Blue Alliance Community (right to left) – IDs 6, 7, 8
    */

    // rotation matrix
    private double cameraTilt= 0.0 * Math.PI / 180.0;
    private double[] cameraOffset = {0.0, 0.0, 0.0}; // goes {x, y, z} // In inches // TODO: figure this offset

    private double sinTheta;
    private double cosTheta;

    public DetectionList yoloObjects;
    public DetectionList aprilTags;

	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	public ObjectTrackerSubsystem(String source){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.source = source; 
        monsterVision = inst.getTable("MonsterVision");
        jsonString = "";

        if (source == "NoteCam") { // is it defined eclipse?
            cameraTilt= Constants.VISION_NOTE_CAM_TILT;
        }
        else if(source == "AprilTagPro"){
            cameraTilt=Constants.VISION_APRIL_TAG_PRO_TILT;
        }

        sinTheta = Math.sin(cameraTilt);
        cosTheta = Math.cos(cameraTilt);

        yoloObjects = new DetectionList();
        aprilTags = new DetectionList();

        // monsterVision.addEntryListener(
        //     "ObjectTracker",
        //     (monsterVision, key, entry, value, flags) -> {
        //    System.out.println("ObjectTracker changed value: " + value.getValue());
        // }, 
        // EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);          
    }
    
    
    public void data() {
        NetworkTableEntry entry = monsterVision.getEntry("ObjectTracker-" + source);
        if(entry==null) {
            return;
        }
        // default to an empty list of detections if nothing is found: 
        jsonString = entry.getString("[]");
        // TODO: call updateDetections with detectionsString = jsonString to populate yoloObjects and aprilTags
        updateDetections(jsonString, gson);
        // use the getClosestAprilTag() to get the detection for the closest april tag
        // Use smart dashboarf to display the x, y, z and ya values
        try {
            SmartDashboard.putNumber("VisionX", getNearestAprilTagDetection().x);
            SmartDashboard.putNumber("VisionY", getNearestAprilTagDetection().y);
            SmartDashboard.putNumber("VisionZ", getNearestAprilTagDetection().z);
            SmartDashboard.putNumber("VisionYa", getNearestAprilTagDetection().ya);
            //System.out.println("x: "+ getNearestAprilTagDetection().x + ", y: "+ getNearestAprilTagDetection().y + ", z: " + getNearestAprilTagDetection().z + ", ya: "+ getNearestAprilTagDetection().ya);
            visionZ = getNearestAprilTagDetection().z;
            visionX = getNearestAprilTagDetection().x;
            visionY = getNearestAprilTagDetection().y;
            String fpsString = monsterVision.getEntry("ObjectTracker-fps").getString("").substring(5);
            double fps = Double.valueOf(fpsString);
            SmartDashboard.putNumber("CameraFPS", fps);
        } catch (Exception e) {
            System.out.println(e);
        }
        return ;
        /* This commented code uses the OLD VisionObject
        try {
            foundObjects = gson.fromJson(jsonString, VisionObject[].class);
        } catch (Exception e) {
            foundObjects = null; 
        }

        // loop over list of visionobjects, deletes them from list if z=0
        // this handles case found on 3/22 where a cone is (0, 0, 0) despite being far away
        try {
            ArrayList<VisionObject> tmp = new ArrayList<VisionObject>(Arrays.asList(foundObjects));
            for (int i = 0; i < tmp.size(); i++) {
                VisionObject vo = tmp.get(i);
                if (vo.z == 0) {
                    tmp.remove(vo);
                }
            }
            // convert arraylist to array via for loop bc .toArray() is being uncooperative
            VisionObject[] tmp2 = new VisionObject[tmp.size()];
            for (int i = 0; i<tmp.size();i++){
                tmp2[i] = tmp.get(i);
            }
            foundObjects = tmp2;
        } catch (Exception e) {
            foundObjects = null;
        }
        
        if (foundObjects != null && source.contains("Chassis")) {
            applyRotationTranslationMatrix();
        }
        // TODO: Comment this part
        // for (VisionObject object : foundObjects){
        //     System.out.format("%s %.1f %.1f %.1f %.1f\n",object.objectLabel, object.x, object.y, object.z, object.confidence);
        // }      '' 
        */
    }

    private void applyRotationTranslationMatrix() {
        // sets reference to be the CENTER of the robot 
        
        for (int i = 0; i < foundObjects.length; i++) {
            double x = foundObjects[i].x; 
            double y = foundObjects[i].y; 
            double z = foundObjects[i].z; 

            // rotation + translation
            foundObjects[i].x = x + cameraOffset[0]; 
            foundObjects[i].y = y * cosTheta - z * sinTheta + cameraOffset[1]; 
            foundObjects[i].z = y * sinTheta + z * cosTheta + cameraOffset[2];
        }
    }
    
    public String getObjectsJson()
    {
        return jsonString;
    }
    
    // private NetworkTableEntry getEntry(Integer index, String subkey) {
    //     try {
    //         NetworkTable table = monsterVision.getSubTable(index.toString());
    //         NetworkTableEntry entry = table.getEntry(subkey);
    //         return entry;
    //     }
    //     catch (Exception e){
    //         return null;
    //     } 
    // }
	
	public VisionObject getClosestObject(String objectLabel) {

        VisionObject[] objects = getObjectsOfType(objectLabel);
        if (objects == null || objects.length == 0) {
            return null; 
        }
        return objects[0];
    }

    public VisionObject getClosestObject() {
        VisionObject[] objects = getObjects(0.5);
        if (objects == null || objects.length == 0) {
            return null; 
        }
        return objects[0];
    }

    public VisionObject getSecondClosestObject(String objectLabel) {
        VisionObject[] objects = getObjectsOfType(objectLabel);
        if (objects == null || objects.length == 0) {
            return null; 
        }
        return objects[1];
    }

    /** Returns closest AprilTag */
    public VisionObject getClosestAprilTag() {
        VisionObject object = getClosestObject("tag");
        if (object == null) {
            return null; 
        }
        return object;
    }

    public VisionObject getSpecificAprilTag(int id) {
        String objectLabel = "tag16h5: " + id;
        VisionObject[] objects = getObjectsOfType(objectLabel);
        if (objects == null || objects.length == 0) {
            return null; 
        }
        return objects[0];
    }


    /** Returns whether closest cone/cube to the gripper if close enough to pick up 
     * @param isCube TRUE cube, FALSE cone
    */
    public boolean isGripperCloseEnough(boolean isCube) {
        // this target is the target y value when the object moves between the claws for pick up
        double targetY = isCube ? 5 : 2; //TODO: figure this y position out (somehting <0 bc its below the cneter of the FOV)
        double actualY = 0; //TODO: get current y of the object

        return actualY < targetY; // TODO may want to change min based on whether it's a cube or cone
    }
    

    public int numberOfObjects() {
        return foundObjects.length; 
    }
    
    public VisionObject[] getObjects(double minimumConfidence) {

        if (foundObjects == null || foundObjects.length == 0)
            return null;

        List<VisionObject> filteredResult = Arrays.asList(foundObjects)
            .stream()
            .filter(vo -> vo.confidence >= minimumConfidence )
            .collect(Collectors.toList());

        VisionObject filteredArray[] = new VisionObject[filteredResult.size()];
        return filteredResult.toArray(filteredArray);

    }

    public VisionObject[] getObjectsOfType(String objectLabel) {
        if (foundObjects == null || foundObjects.length == 0)
            return null;
        List<VisionObject> filteredResult = Arrays.asList(foundObjects)
            .stream()
            .filter(vo -> vo.objectLabel.contains(objectLabel) && (objectLabel.contains("tag") || vo.confidence > .40))//Uses .contains because vo.ObjectLabel has ID, ObjectLabel does not
            .collect(Collectors.toList());


            VisionObject filteredArray[] = new VisionObject[filteredResult.size()];
        return filteredResult.toArray(filteredArray);

    }
    public void saveVisionSnapshot(String fileName) 
    throws IOException {
        data();    
        Gson gson = new Gson();
        String str = gson.toJson(foundObjects);
        BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
        writer.write(str);
        
        writer.close();
    }

    public VisionObject[] loadVisionSnapshot(String fileName) 
    throws IOException {  
        Path filePath = Path.of(fileName);
        Gson gson = new Gson();

        String json = Files.readString(filePath);
        VisionObject[] snapShotObjects = gson.fromJson(json, VisionObject[].class);
        
        return snapShotObjects;
    }


    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

// ================================================================================
// Functions from MyClass from JDoodle

    public Detection getNearestAprilTagDetection() {
        if (aprilTags.size() > 0) {
            return aprilTags.get(0);
        }
        return null;
    }
    public Detection[] getNearestAprilTagsDetection(int count) {
        Detection[] detections = new Detection[count];
        if (count <= aprilTags.size()) {
            for (int i = 0; i < count; i++) {
                detections[i] = aprilTags.get(i);
            }
            return detections;
        } else {
            return null;
        }
    }
    public Detection getNearestYoloDetection() {
        if (yoloObjects.size() > 0) {
            return yoloObjects.get(0);
        }
        return null;
    }
    public Detection[] getNearestYoloDetections(int count) {
        Detection[] detections = new Detection[count];
        if (count <= yoloObjects.size()) {
            for (int i = 0; i < count; i++) {
                detections[i] = yoloObjects .get(i);
            }
            return detections;
        } else {
            return null;
        }
    }
    // // Yaw difference is from the frame of reference of rotation
    // // Get Y from the rotaion object of detection and translate to z rotation of robot spo we can directly feed into command to turn robot
    // // Confirm that we are turning in correct direction
    // public double getYawDifferenceFromDetectionRotation(Detection detection) {
    //     // The whole point of this functions is so that we know which direction to turn the robot
    //     return 0.0;
    // }
    // public double getDistanceFromDetection(Detection detection) {
    //     return detection.z;
    // }
    // public double getXFieldDistanceFromDetection(Detection detection) {
    //     // x not
    //     return 0.0;
    // }
    private double getThetaYZField(Detection detection) {
        double camX = detection.x;
        double camZ = detection.z;
        double yCamAngle = detection.ya;
        double thetaYZ = Math.tanh(camZ/camX);
        double thetaYZField = 90.0 - yCamAngle - thetaYZ;

        return thetaYZField;
    }
    public double getYFieldAprilFromDetection(Detection detection) {
        double yField;

        double radius = getRadius(detection);
        double thetaYZField = getThetaYZField(detection);

        yField = Math.cos(thetaYZField) * radius;

        return yField;
    }
    public double getXFieldAprilFromDetection(Detection detection) {
        double xField;

        double radius = getRadius(detection);
        double thetaYZField = getThetaYZField(detection);

        xField = Math.sin(thetaYZField) * radius;

        return xField;
    }
    public double getRadius(Detection detection) {
        double camX = detection.x;
        double camZ = detection.z;
        
        double radius = Math.sqrt(Math.pow(camX, 2) + Math.pow(camZ, 2));
        return radius;
    }
    public void updateDetections(String detectionsString, Gson gson) {
        DetectionList gsonOut = gson.fromJson(detectionsString, DetectionList.class);
        aprilTags.clear();
        yoloObjects.clear();
        
        // Seperate out the detections with rotation
        for (int i = 0; i < gsonOut.size(); i++) { // Maybe change later
            if (gsonOut.get(i).objectLabel.substring(0,3).equals("tag")) {
                aprilTags.add(gsonOut.get(i));
                // System.out.println("UpdateDetections(: found apriltag");
            } else {
                yoloObjects.add(gsonOut.get(i));
                // System.out.println("yolo object");
            }
        }
    }


}