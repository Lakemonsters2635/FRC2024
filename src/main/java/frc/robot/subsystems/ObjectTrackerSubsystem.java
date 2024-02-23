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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.models.VisionObject;


public class ObjectTrackerSubsystem extends SubsystemBase {
	  NetworkTable networkTable; 
    public VisionObject[] foundObjects; 
    private String jsonString;
    private String source;
    private Gson gson = new Gson();


    /*
     * Red Alliance Community (right to left) – IDs 1, 2, 3
     * Blue Alliance Double Substation – ID 4
     * Red Alliance Double Substation – ID 5
     * Blue Alliance Community (right to left) – IDs 6, 7, 8
    */

    // rotation matrix
    private double cameraTilt = 0.0 * Math.PI / 180.0; //Update this      TODO: update
    private double[] cameraOffset = {-2.0, 0.0, 0.0}; // goes {x, y, z} // TODO: figure this offset

    private double sinTheta = Math.sin(cameraTilt);
    private double cosTheta = Math.cos(cameraTilt);

	// Put methods for controlling this subsystem
  // here. Call these from Commands.
	public ObjectTrackerSubsystem(String source, String networkTableKey){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.source = source; 
        networkTable = inst.getTable(networkTableKey);
        jsonString = "";
        // networkTable.addEntryListener(
        //     "ObjectTracker",
        //     (networkTable, key, entry, value, flags) -> {
        //    System.out.println("ObjectTracker changed value: " + value.getValue());
        // }, 
        // EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);          
    }
    
    
    public void data() {
        NetworkTableEntry entry = networkTable.getEntry("ObjectTracker-" + source);
        if(entry==null) {
            return;
        }
        jsonString = entry.getString("ObjectTracker");
        
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
        // for (VisionObject object : foundObjects){
        //     System.out.format("%s %.1f %.1f %.1f %.1f\n",object.objectLabel, object.x, object.y, object.z, object.confidence);
        // }       
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
    //         NetworkTable table = networkTable.getSubTable(index.toString());
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




}
